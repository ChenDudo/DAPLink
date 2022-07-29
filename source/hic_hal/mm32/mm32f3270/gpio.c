/**
 * @file    gpio.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
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
#include "hal_gpio.h"
#include "hal_rcc.h"
#include "DAP_config.h"
#include "gpio.h"
#include "daplink.h"
#include "util.h"
#include "beep.h"

#include "settings.h"

static void busy_wait(uint32_t cycles)
{
    volatile uint32_t i;
    i = cycles;

    while (i > 0)
    {
        i--;
    }
}

void Power_5v_En(void)
{
    GPIO_ResetBits(POWER_3V3_EN_PIN_PORT, POWER_3V3_EN_PIN);
    GPIO_SetBits(POWER_5V_EN_PIN_PORT, POWER_5V_EN_PIN);
}

void Power_3v3_En(void)
{
    GPIO_ResetBits(POWER_5V_EN_PIN_PORT, POWER_5V_EN_PIN);
    GPIO_SetBits(POWER_3V3_EN_PIN_PORT, POWER_3V3_EN_PIN);
}

void Power_Off(void)
{
    GPIO_ResetBits(POWER_5V_EN_PIN_PORT, POWER_5V_EN_PIN);
    GPIO_ResetBits(POWER_3V3_EN_PIN_PORT, POWER_3V3_EN_PIN);
}

void Power_Supply(void)
{
    if (config_get_power_output()){
		config_get_5v_output() ? Power_5v_En() : Power_3v3_En();
	}
	else
		Power_Off();
}

void Beep_En(bool enable)
{
    enable ? BEEP_on() : BEEP_off();
}

void LED_off(void)
{
#if defined (LED1_PORT)
    GPIO_SetBits(LED1_PORT, LED1_PIN);
#endif
#if defined (LED2_PORT)
    GPIO_SetBits(LED2_PORT, LED2_PIN);
#endif
#if defined (LED3_PORT)
    GPIO_SetBits(LED3_PORT, LED3_PIN);
#endif
}

void LED_on(void)
{
#if defined (LED1_PORT)    
    GPIO_ResetBits(LED1_PORT, LED1_PIN);
#endif
#if defined (LED2_PORT)    
    GPIO_ResetBits(LED2_PORT, LED2_PIN);
#endif
#if defined (LED3_PORT)    
    GPIO_ResetBits(LED3_PORT, LED3_PIN);
#endif
}

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // enable clock to ports
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // configure LEDs
    GPIO_ResetBits(RUNNING_LED_PORT, RUNNING_LED_PIN);
    GPIO_InitStructure.GPIO_Pin = RUNNING_LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RUNNING_LED_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(CONNECTED_LED_PORT, CONNECTED_LED_PIN);
    GPIO_InitStructure.GPIO_Pin = CONNECTED_LED_PIN;
    GPIO_Init(CONNECTED_LED_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(PIN_CDC_LED_PORT, PIN_CDC_LED);
    GPIO_InitStructure.GPIO_Pin = PIN_CDC_LED;
    GPIO_Init(PIN_CDC_LED_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(PIN_MSC_LED_PORT, PIN_MSC_LED);
    GPIO_InitStructure.GPIO_Pin = PIN_MSC_LED;
    GPIO_Init(PIN_MSC_LED_PORT, &GPIO_InitStructure);
	
    // configure reset
    GPIO_PinAFConfig(SWCLK_TCK_PIN_PORT, SWCLK_TCK_PIN_Bit, GPIO_AF_0);
    SWCLK_TCK_PIN_PORT->BRR = SWCLK_TCK_PIN;
    GPIO_InitStructure.GPIO_Pin = SWCLK_TCK_PIN;
    GPIO_Init(SWCLK_TCK_PIN_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN_Bit, GPIO_AF_0);
    SWDIO_OUT_PIN_PORT->BRR = SWDIO_OUT_PIN;
    GPIO_InitStructure.GPIO_Pin = SWDIO_OUT_PIN;
    GPIO_Init(SWDIO_OUT_PIN_PORT, &GPIO_InitStructure);
#if defined(SWDIO_DIR_PIN_PORT)
    SWDIO_DIR_PIN_PORT->BRR = SWDIO_DIR_PIN;
    GPIO_InitStructure.GPIO_Pin = SWDIO_DIR_PIN;
    GPIO_Init(SWDIO_DIR_PIN_PORT, &GPIO_InitStructure);
#endif

#if defined(nRST_DIR_PIN_PORT)
	GPIO_PinAFConfig(nRST_DIR_PIN_PORT, nRST_DIR_PIN_Bit, 0);
    GPIO_SetBits(nRST_DIR_PIN_PORT, nRST_DIR_PIN);
    GPIO_InitStructure.GPIO_Pin = nRST_DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(nRST_DIR_PIN_PORT, &GPIO_InitStructure);
#endif
    // nRESET OUT
	GPIO_PinAFConfig(nRESET_PIN_PORT, nRESET_PIN_Bit, 0);
    GPIO_SetBits(nRESET_PIN_PORT, nRESET_PIN);
    GPIO_InitStructure.GPIO_Pin = nRESET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(nRESET_PIN_PORT, &GPIO_InitStructure);
	
// K1
#if defined(K1_PIN_PORT)
    GPIO_SetBits(K1_PIN_PORT, K1_PIN);
    GPIO_InitStructure.GPIO_Pin = K1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(K1_PIN_PORT, &GPIO_InitStructure);
#endif

    /* Turn on power to the board. */
    // When the target is unpowered, it holds the reset line low.
    Power_Supply();

    GPIO_PinAFConfig(POWER_3V3_EN_PIN_PORT, POWER_3V3_EN_Bit, POWER_3V3_EN_AF);
    GPIO_PinAFConfig(POWER_5V_EN_PIN_PORT, POWER_5V_EN_Bit, POWER_5V_EN_AF);
    GPIO_InitStructure.GPIO_Pin		= POWER_5V_EN_PIN;
    GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_Out_PP;
    GPIO_Init(POWER_5V_EN_PIN_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin		= POWER_3V3_EN_PIN;
    GPIO_Init(POWER_3V3_EN_PIN_PORT, &GPIO_InitStructure);

    InitBeep();
	if(config_get_beep_en()){
		beepEn = true;
		BEEP_on();
	}
	else {
		beepEn = false;
		BEEP_off();
	}
		
    // Let the voltage rails stabilize.  This is especailly important
    // during software resets, since the target's 3.3v rail can take
    // 20-50ms to drain.  During this time the target could be driving
    // the reset pin low, causing the bootloader to think the reset
    // button is pressed.
    // Note: With optimization set to -O2 the value 1000000 delays for ~85ms
    LED_on();
    busy_wait(1000000);
    LED_off();
    BEEP_off();
	beepMode = modeN; beepCount = 0;
}

void gpio_set_hid_led(gpio_led_state_t state)
{
    if (state)
    {
        GPIO_ResetBits(PIN_HID_LED_PORT, PIN_HID_LED); // LED on
    }
    else
    {
        GPIO_SetBits(PIN_HID_LED_PORT, PIN_HID_LED); // LED off
    }
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    if (state)
    {
        GPIO_ResetBits(PIN_CDC_LED_PORT, PIN_CDC_LED); // LED on
    }
    else
    {
        GPIO_SetBits(PIN_CDC_LED_PORT, PIN_CDC_LED); // LED off
    }
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    if (state)
    {
        GPIO_ResetBits(PIN_MSC_LED_PORT, PIN_MSC_LED); // LED on
    }
    else
    {
        GPIO_SetBits(PIN_MSC_LED_PORT, PIN_MSC_LED); // LED off
    }
}

uint8_t gpio_get_reset_btn_no_fwrd(void)
{
#if defined(K1_PIN_PORT)
    return (K1_PIN_PORT->IDR & K1_PIN) ? 0 : 1;
#else
    return 0;
#endif
}

uint8_t gpio_get_reset_btn_fwrd(void)
{
    return 0;
}

uint8_t GPIOGetButtonState(void)
{
    return 0;
}

void target_forward_reset(bool assert_reset)
{
    // Do nothing - reset is forwarded in gpio_get_sw_reset
}

void gpio_set_board_power(bool powerEnabled)
{
}
