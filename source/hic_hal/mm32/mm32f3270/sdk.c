/**
 * @file    sdk.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2017-2017, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mm32_device.h"
#include "reg_common.h"

#include "DAP_config.h"
#include "gpio.h"
#include "daplink.h"
#include "util.h"
#include "cortex_m.h"

uint32_t SystemCoreClock;

////////////////////////////////////////////////////////////////////////////////
static uint32_t AutoCalPllFactor(uint32_t pllclkSourceFrq, uint32_t pllclkFrq, uint8_t* plln, uint8_t* pllm)
{
    uint32_t n, m;
    uint32_t tempFrq;
    uint32_t minDiff = pllclkFrq;
    uint8_t  flag = 0;
    for(m = 0; m < 4 ; m++) {
        for(n = 0; n < 64 ; n++) {
            tempFrq =  pllclkSourceFrq * (n + 1) / (m + 1);
            tempFrq = (tempFrq >  pllclkFrq) ? (tempFrq - pllclkFrq) : (pllclkFrq - tempFrq) ;

            if(minDiff > tempFrq) {
                minDiff =  tempFrq;
                *plln = n;
                *pllm = m;
            }
            if(minDiff == 0) {
                flag = 1;
                break;
            }
        }
        if(flag != 0) {
            break;
        }
    }
    return  minDiff;
}

////////////////////////////////////////////////////////////////////////////////
//static void DELAY_xUs(uint32_t count)
//{
//    uint32_t temp;
//    SysTick->CTRL = 0x0;                                                        //disable systick function
//    SysTick->LOAD = count * 8;                                                    //time count for 1us with HSI as SYSCLK
//    SysTick->VAL = 0x00;                                                        //clear counter
//    SysTick->CTRL = 0x5;                                                        //start discrease with Polling
//    do {
//        temp = SysTick->CTRL;
//    } while((temp & 0x01) && !(temp & (1 << 16)));                              //wait time count done
//    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;                                  //Close Counter
//    SysTick->VAL = 0X00;                                                        //clear counter
//}

/**
  * @brief  Switch the PLL source from HSI to HSE bypass, and select the PLL as SYSCLK
  *         source.
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE bypass)
  *            SYSCLK(Hz)                     = 96 MHz
  *            HCLK(Hz)                       = 96 MHZ
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8 MHz
  *            HSE PREDIV1                    = 0+1
  *            PLLMUL                         = 11+1
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
void sdk_init()
{
	/* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers, 
	Configure the Flash Latency cycles and enable prefetch buffer
	*/
    __IO uint32_t tn, tm;
    __IO uint32_t StartUpCounter = 0;
	uint8_t plln, pllm;
	
	SystemCoreClock = 96000000;
	
	CACHE->CCR &= ~(0x3 << 3);
    CACHE->CCR |= 1;
    while(2 != (CACHE->SR & 0x3));

    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);

    // DELAY_xUs(5);
    if(SystemCoreClock > 96000000) {
        RCC->APB1ENR |= RCC_APB1ENR_PWR;
        PWR->CR &= ~(3 << 14);
        PWR->CR |= 3 << 14;
    }
	
    //Wait till HSE is ready and if Time out is reached exit
    while (1){
        if ((RCC->CR & RCC_CR_HSERDY))	break;
        StartUpCounter++;
        if(StartUpCounter >= (10 * HSE_STARTUP_TIMEOUT)) return;
    }
    if (!(RCC->CR & RCC_CR_HSERDY))	return;	//ERROR: HSE fails to start-up
    // DELAY_xUs(5);
	
    //Enable Prefetch Buffer
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_3;
	
	//Configure HCLK, PCLK2 and PCLK1 prescalers
    RCC->CFGR &= (~RCC_CFGR_HPRE) & ( ~RCC_CFGR_PPRE1) & (~RCC_CFGR_PPRE2);
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

	//Configure PLL
    RCC->PLLCFGR &= ~((uint32_t) RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLXTPRE) ;
    RCC->PLLCFGR |= (uint32_t) RCC_PLLCFGR_PLLSRC;
    
	AutoCalPllFactor(HSE_VALUE, SystemCoreClock, &plln, &pllm);
    tm = (((uint32_t)pllm) & 0x07);
    tn = (((uint32_t)plln) & 0x7F);
    
	RCC->APB1ENR |= RCC_APB1ENR_PWR;
    RCC->PLLCFGR &= (uint32_t)((~RCC_PLLCFGR_PLL_DN) & (~RCC_PLLCFGR_PLL_DP));
    RCC->PLLCFGR |= ((tn << RCC_PLLCFGR_PLL_DN_Pos) | (tm << RCC_PLLCFGR_PLL_DP_Pos));
   
	//Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY)) {__ASM ("nop");}//__NOP();
	
    //Select PLL as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    //Wait till PLL is used as system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {__ASM ("nop");} //__NOP();
    //DELAY_xUs(1);
	
	//Configure USB prescaler
	RCC->CFGR &= ~(uint32_t)RCC_CFGR_USBPRE;
	RCC->CFGR |= 1 << RCC_CFGR_USBPRE_Pos;	// div = 1
}
