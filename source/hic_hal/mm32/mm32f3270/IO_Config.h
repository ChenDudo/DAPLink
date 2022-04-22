/**
 * @file    IO_Config.h
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

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "mm32_device.h"
#include "compiler.h"
#include "daplink.h"

#include "hal_gpio.h"

COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_MM32F3270);

//USB control pin
//#define USB_CONNECT_PORT_ENABLE()    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE)
//#define USB_CONNECT_PORT_DISABLE()   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE)
//#define USB_CONNECT_PORT             GPIOA
//#define USB_CONNECT_PIN              GPIO_Pin_15
//#define USB_CONNECT_ON()             (USB_CONNECT_PORT->BSRR = USB_CONNECT_PIN)
//#define USB_CONNECT_OFF()            (USB_CONNECT_PORT->BRR  = USB_CONNECT_PIN)


//When bootloader, disable the target port(not used)
#define POWER_5V_EN_PIN_PORT         GPIOC
#define POWER_5V_EN_PIN              GPIO_Pin_10
#define POWER_5V_EN_Bit              10

// AF0: JTDI ....    AF2/3 is null
#define POWER_3V3_EN_PIN_PORT        GPIOA
#define POWER_3V3_EN_PIN             GPIO_Pin_15
#define POWER_3V3_EN_Bit             15

// nRESET OUT Pin
#define nRESET_PIN_PORT              GPIOC
#define nRESET_PIN                   GPIO_Pin_12
#define nRESET_PIN_Bit               12
// nRESET IN Pin (K1)
#define K1_PIN_PORT                 GPIOD
#define K1_PIN                      GPIO_Pin_2
#define K1_PIN_Bit                  2

//SWD
// SWCLK: spi2_SCK
#define SWCLK_TCK_PIN_PORT           GPIOB
#define SWCLK_TCK_PIN                GPIO_Pin_13
#define SWCLK_TCK_PIN_Bit            13

// SWDIO3: spi1_MOSI 
// SWDIO1: GPIO
#define SWDIO_OUT_PIN_PORT           GPIOB
#define SWDIO_OUT_PIN                GPIO_Pin_5
#define SWDIO_OUT_PIN_Bit            5

// SWDIO2: spi1_MISO
#define SWDIO_IN_PIN_PORT            GPIOA
#define SWDIO_IN_PIN                 GPIO_Pin_6
#define SWDIO_IN_PIN_Bit             6


// LEDs
#define LED1_PORT             GPIOB
#define LED1_PIN              GPIO_Pin_14
#define LED1_Bit              14

#define LED2_PORT             GPIOB
#define LED2_PIN              GPIO_Pin_10
#define LED2_Bit              10

#define LED3_PORT             GPIOB
#define LED3_PIN              GPIO_Pin_1
#define LED3_Bit              1

//Connected LED
#define RUNNING_LED_PORT             LED1_PORT
#define RUNNING_LED_PIN              LED1_PIN
#define RUNNING_LED_Bit              LED1_Bit

#define CONNECTED_LED_PORT           LED2_PORT
#define CONNECTED_LED_PIN            LED2_PIN
#define CONNECTED_LED_PIN_Bit        LED2_Bit

//USB status LED
#define PIN_HID_LED_PORT             LED3_PORT
#define PIN_HID_LED                  LED3_PIN
#define PIN_HID_LED_Bit              LED3_Bit

#define PIN_CDC_LED_PORT             LED3_PORT
#define PIN_CDC_LED                  LED3_PIN
#define PIN_CDC_LED_Bit              LED3_Bit

#define PIN_MSC_LED_PORT             LED3_PORT
#define PIN_MSC_LED                  LED3_PIN
#define PIN_MSC_LED_Bit              LED3_Bit


//Beep
#define BEEP_PORT             GPIOB
#define BEEP_PIN              GPIO_Pin_8
#define BEEP_Bit              8

#endif
