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

#if defined(MM32LINK_MAX) 
COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_MM32_MB059);
#elif defined(MM32LINK_MINI) 
COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_MM32_MB088);
#else
COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_MM32_OB);
#endif

// When bootloader, disable the target port(not used)

#if defined(MM32LINK_MAX) /* MAX */
// 1. Power
#define POWER_5V_EN_PIN_PORT    GPIOC
#define POWER_5V_EN_Bit         (10)
#define POWER_5V_EN_PIN         (1 << POWER_5V_EN_Bit)
#define POWER_5V_EN_AF          GPIO_AF_0

#define POWER_3V3_EN_PIN_PORT   GPIOA
#define POWER_3V3_EN_Bit        (15)
#define POWER_3V3_EN_PIN        (1 << POWER_3V3_EN_Bit)
#define POWER_3V3_EN_AF         GPIO_AF_2

// 2. nRESET OUT
#define nRESET_PIN_PORT         GPIOC
#define nRESET_PIN_Bit          (12)
#define nRESET_PIN              (1 << nRESET_PIN_Bit)

// 3. K1
#define K1_PIN_PORT             GPIOD
#define K1_PIN_Bit              (2)
#define K1_PIN                  (1 << K1_PIN_Bit)

// 4. SWD CLK
#define SWCLK_TCK_PIN_PORT      GPIOA
#define SWCLK_TCK_PIN_Bit       (5)
#define SWCLK_TCK_PIN           (1 << SWCLK_TCK_PIN_Bit)

// 5. SWDIO
// SWDIO3: spi1_MOSI
// SWDIO1: GPIO (yes)
#define SWDIO_OUT_PIN_PORT      GPIOA
#define SWDIO_OUT_PIN_Bit       (4)
#define SWDIO_OUT_PIN           (1 << SWDIO_OUT_PIN_Bit)

// SWDIO2: spi1_MISO
#define SWDIO_IN_PIN_PORT       GPIOA
#define SWDIO_IN_PIN_Bit        (6)
#define SWDIO_IN_PIN            (1 << SWDIO_IN_PIN_Bit)

// 6. SWO/TDO
#define SWDO_PIN_PORT           GPIOC
#define SWDO_PIN_Bit            (11)
#define SWDO_PIN                (1 << SWDO_PIN_Bit)

// 7. TDI PC14
#define TDI_PIN_PORT            GPIOC
#define TDI_PIN_Bit             (14)
#define TDI_PIN                 (1 << SWDO_PIN_Bit)

// 8. nJTRST PB4
#define nJTRST_PIN_PORT         GPIOB
#define nJTRST_PIN_Bit          (4)
#define nJTRST_PIN              (1 << SWDO_PIN_Bit)

// 8. LEDs
#define LED1_PORT               GPIOB
#define LED1_PIN_Bit			(14)
#define LED1_PIN                (1 << LED1_PIN_Bit)

#define LED2_PORT               GPIOB
#define LED2_PIN_Bit			(10)
#define LED2_PIN                (1 << LED2_PIN_Bit)

#define LED3_PORT               GPIOB
#define LED3_PIN_Bit            (1)
#define LED3_PIN                (1 << LED3_PIN_Bit)

#elif defined(MM32LINK_MINI) /* MINI */
#if 1
// 1. Power: 					PB12(5V) / PB13(3.3V)
#define POWER_5V_EN_PIN_PORT    GPIOB
#define POWER_5V_EN_Bit         (12)
#define POWER_5V_EN_PIN         (1 << POWER_5V_EN_Bit)
#define POWER_5V_EN_AF          GPIO_AF_0

#define POWER_3V3_EN_PIN_PORT   GPIOB
#define POWER_3V3_EN_Bit        (13)
#define POWER_3V3_EN_PIN        (1 << POWER_3V3_EN_Bit)
#define POWER_3V3_EN_AF         GPIO_AF_0

// 2. nRESET OUT/IN: 			PA4
#define nRESET_PIN_PORT         GPIOA
#define nRESET_PIN_Bit          (4)
#define nRESET_PIN              (1 << nRESET_PIN_Bit)

// nRST DIR: 					PB10
#define nRST_DIR_PIN_PORT       GPIOB
#define nRST_DIR_PIN_Bit        (10)
#define nRST_DIR_PIN            (1 << nRST_DIR_PIN_Bit)

// 3. K1


// 4. SWD CLK: 					PA5
#define SWCLK_TCK_PIN_PORT      GPIOA
#define SWCLK_TCK_PIN_Bit       (5)
#define SWCLK_TCK_PIN           (1 << SWCLK_TCK_PIN_Bit)

// 5. SWDIO
// SWDIO1 out:					PA7
#define SWDIO_OUT_PIN_PORT      GPIOA
#define SWDIO_OUT_PIN_Bit       (7)
#define SWDIO_OUT_PIN           (1 << SWDIO_OUT_PIN_Bit)

// SWDIO2 in(MISO)				PA6
#define SWDIO_IN_PIN_PORT       GPIOA
#define SWDIO_IN_PIN_Bit        (6)
#define SWDIO_IN_PIN            (1 << SWDIO_IN_PIN_Bit)

// SWDIO DIR: 					PB0
#define SWDIO_DIR_PIN_PORT      GPIOB
#define SWDIO_DIR_PIN_Bit       0
#define SWDIO_DIR_PIN           (1 << SWDIO_DIR_PIN_Bit)

// 6. SWO/TDO: 					PB1
#define SWDO_PIN_PORT           GPIOB
#define SWDO_PIN_Bit            (1)
#define SWDO_PIN                (1 << SWDO_PIN_Bit)
#define SWDO_AF                 GPIO_AF_8

// 7. LEDs PA0 / PA1
#define LED1_PORT               GPIOA
#define LED1_PIN_Bit			(1)
#define LED1_PIN                (1 << LED1_PIN_Bit)

#define LED2_PORT               GPIOA
#define LED2_PIN_Bit        	(0)
#define LED2_PIN                (1 << LED2_PIN_Bit)

// 8. ADC PA2 / PA3
#define DET_TVDD_PORT          	GPIOA
#define DET_TVDD_PIN_Bit		(2)
#define DET_TVDD_PIN           	(1 << DET_TVDD_PIN_Bit)

#define DET_TVCC_PORT          	GPIOA
#define DET_TVCC_PIN_Bit		(3)
#define DET_TVCC_PIN           	(1 << DET_TVCC_PIN_Bit)

//#else
#else
// 1. Power: 					PC14(5V) / PC13(3.3V)
#define POWER_5V_EN_PIN_PORT    GPIOC
#define POWER_5V_EN_Bit         (14)
#define POWER_5V_EN_PIN         (1 << POWER_5V_EN_Bit)
#define POWER_5V_EN_AF          GPIO_AF_0

#define POWER_3V3_EN_PIN_PORT   GPIOC
#define POWER_3V3_EN_Bit        (13)
#define POWER_3V3_EN_PIN        (1 << POWER_3V3_EN_Bit)
#define POWER_3V3_EN_AF         GPIO_AF_0

// 2. nRESET OUT/IN: 			PA2/PA3
#define nRESET_PIN_PORT         GPIOA
#define nRESET_PIN_Bit          (2)
#define nRESET_PIN              (1 << nRESET_PIN_Bit)

// nRST DIR: 					PB10
//#define nRST_DIR_PIN_PORT       GPIOB
//#define nRST_DIR_PIN_Bit        (10)
//#define nRST_DIR_PIN            (1 << nRST_DIR_PIN_Bit)

// 3. K1


// 4. SWD CLK: 					PB0
#define SWCLK_TCK_PIN_PORT      GPIOB
#define SWCLK_TCK_PIN_Bit       (0)
#define SWCLK_TCK_PIN           (1 << SWCLK_TCK_PIN_Bit)

// 5. SWDIO
// SWDIO1 out:					PA7
#define SWDIO_OUT_PIN_PORT      GPIOA
#define SWDIO_OUT_PIN_Bit       (7)
#define SWDIO_OUT_PIN           (1 << SWDIO_OUT_PIN_Bit)

// SWDIO2 in(MISO)				PA5
#define SWDIO_IN_PIN_PORT       GPIOA
#define SWDIO_IN_PIN_Bit        (5)
#define SWDIO_IN_PIN            (1 << SWDIO_IN_PIN_Bit)

// SWDIO DIR: 					PA6
#define SWDIO_DIR_PIN_PORT      GPIOA
#define SWDIO_DIR_PIN_Bit       6
#define SWDIO_DIR_PIN           (1 << SWDIO_DIR_PIN_Bit)

// 6. SWO/TDO: 					PA4
#define SWDO_PIN_PORT           GPIOA
#define SWDO_PIN_Bit            (4)
#define SWDO_PIN                (1 << SWDO_PIN_Bit)
#define SWDO_AF                 GPIO_AF_8

// 7. LEDs 
#define LED1_PORT               GPIOA
#define LED1_PIN_Bit			(9)
#define LED1_PIN                (1 << LED1_PIN_Bit)

#define LED2_PORT               GPIOA
#define LED2_PIN_Bit        	(9)
#define LED2_PIN                (1 << LED2_PIN_Bit)

// 8. ADC PA2 / PA3
#define DET_TVDD_PORT          	GPIOA
#define DET_TVDD_PIN_Bit		(9)
#define DET_TVDD_PIN           	(1 << DET_TVDD_PIN_Bit)

#define DET_TVCC_PORT          	GPIOA
#define DET_TVCC_PIN_Bit		(9)
#define DET_TVCC_PIN           	(1 << DET_TVCC_PIN_Bit)
#endif

#endif

/****** COMMON  ***************************************************************/

// Connected LED
#define RUNNING_LED_PORT        LED1_PORT
#define RUNNING_LED_PIN         LED1_PIN
#define RUNNING_LED_Bit         LED1_Bit

#define CONNECTED_LED_PORT      LED1_PORT
#define CONNECTED_LED_PIN       LED1_PIN
#define CONNECTED_LED_PIN_Bit   LED1_Bit

// USB status LED
#define PIN_HID_LED_PORT        LED2_PORT
#define PIN_HID_LED             LED2_PIN
#define PIN_HID_LED_Bit         LED2_Bit

#define PIN_CDC_LED_PORT        LED2_PORT
#define PIN_CDC_LED             LED2_PIN
#define PIN_CDC_LED_Bit         LED2_Bit

#if defined(MM32LINK_MAX)
#define PIN_MSC_LED_PORT        LED3_PORT
#define PIN_MSC_LED             LED3_PIN
#define PIN_MSC_LED_Bit         LED3_Bit
#else
#define PIN_MSC_LED_PORT        LED2_PORT
#define PIN_MSC_LED             LED2_PIN
#define PIN_MSC_LED_Bit         LED2_Bit
#endif

// Beep: PB8
#define BEEP_PORT               GPIOB
#define BEEP_PIN_Bit			(8)
#define BEEP_PIN                (1 << BEEP_PIN_Bit)

#endif
