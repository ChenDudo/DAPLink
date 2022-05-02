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

#include "hal_rcc.h"
#include "hal_flash.h"
#include "DAP_config.h"
#include "gpio.h"
#include "beep.h"
#include "daplink.h"
#include "util.h"
#include "cortex_m.h"


void readConfig()
{
	
}

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
  *            HSE PREDIV1                    = 1
  *            PLLMUL                         = 12
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
void sdk_init()
{
	RCC_DeInit();
	CACHE->CCR &= ~(0x3 << 3);
    CACHE->CCR |= 1;
    while((CACHE->SR & 0x3) != 2);
	
	RCC_HSEConfig(RCC_HSE_ON);                                  /* Open HSE */
 	while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);         /* Waiting for the crystal to stabilize the oscillation */

 	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);       /* Enable Flash Prefetch Buffer */
 	FLASH_SetLatency(FLASH_Latency_3);                          /* FLASH Three Latency cycles */

 	RCC_HCLKConfig(RCC_SYSCLK_Div1);                            /* SYSCLK = AHB */
 	RCC_PCLK2Config(RCC_HCLK_Div1);                             /* APB2 = AHB */
 	RCC_PCLK1Config(RCC_HCLK_Div2);                             /* APB1 = AHB/2 */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);             /* USBCLK = PLLCLK/2 */
	RCC_PLLConfig(RCC_HSE_Div1, RCC_PLLMul_12);                 /* 8MHz /1 * 12 = 96MHz */
	RCC_PLLCmd(ENABLE);                                         /* Enable the PLL */

 	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);         /* Check if PLL is locked */
 	RCC_SYSCLKConfig(RCC_PLL);                                  /* Choose SYSTICK is PLLCLK */
 	while(RCC_GetSYSCLKSource() != 0x08);                       /* Check if the PLL clock output is the system clock, 0x08 : SYSCLK is PLL */
	SystemCoreClock = 96000000;
	
	//InitBeep();
	readConfig();
}
