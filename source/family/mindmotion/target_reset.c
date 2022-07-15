/**
 * @file    target_reset.c
 * @brief   Target reset for the new target
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

//#include "target_reset.h"
#include "swd_host.h"
#include "target_family.h"
#include "cmsis_os2.h"

#include "hal_gpio.h"
#include "IO_Config.h"

static void target_before_init_debug_mm32(void)
{
    // any target specific sequences needed before attaching to the DAP across JTAG or SWD 
    swd_write_word((uint32_t)&SCB->AIRCR, ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk));
}

uint8_t readRstInput;
static void prerun_target_config_mm32(void)
{
    /* In some case the CPU will enter "cannot debug" state (low power, SWD pin mux changed, etc.). 
	Doing a hardware reset will clear those states (probably, depends on app). 
	Also, if the external flash's data is not a valid bootable image, DAPLink cannot attached to target. 
	A hardware reset will increase the chance to connect in this situation. */

	// nReset out mode: Low level
	GPIO_ResetBits(nRESET_PIN_PORT, nRESET_PIN);

	// nReset Dir Output
// #if defined(nRST_DIR_PIN_PORT)	
// 	GPIO_SetBits(nRST_DIR_PIN_PORT, nRST_DIR_PIN);
// #endif
	
	osDelay(4);

	// nReset input mode: Pull up input
	GPIO_SetBits(nRESET_PIN_PORT, nRESET_PIN);
    // nReset Dir Input
// #if defined(nRST_DIR_PIN_PORT)
// 	GPIO_ResetBits(nRST_DIR_PIN_PORT, nRST_DIR_PIN);
// #endif

	osDelay(2);
}

/*
static void swd_set_target_reset_mm32(uint8_t asserted)
{
	if (asserted)
		swd_write_word((uint32_t)&SCB->AIRCR, ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk));
}

static uint8_t target_unlock_sequence(void)
{
    // if the device can secure the flash and there is a way to erase all it should be implemented here.
    return 1;
}

static uint8_t target_set_state(target_state_t state)
{
    // if a custom state machine is needed to set the TARGET_RESET_STATE state
    return 1;
}
*/

//static uint8_t security_bits_set(uint32_t addr, uint8_t *data, uint32_t size)
//{
//    /* if there are security bits in the programmable flash region a check should be performed. 
//	This method is used when programming by drag-n-drop and should refuse to program an image requesting to set the device security. 
//	This can be performed with the debug channel if needed. */
//    return 0;
//}


const target_family_descriptor_t g_target_family_mm32 = {
    //.family_id                  = kMindMotion_FamilyID,
    .default_reset_type         = kHardwareReset,	//kHardwareReset,
    .soft_reset_type            = SYSRESETREQ,
    .target_before_init_debug   = target_before_init_debug_mm32,
    .prerun_target_config       = prerun_target_config_mm32,
    //.target_unlock_sequence     = target_unlock_sequence,
    //.security_bits_set          = security_bits_set,
    //.target_set_state           = target_set_state,
    //.swd_set_target_reset       = swd_set_target_reset_mm32,
    //.validate_bin_nvic          = ,
    //.validate_hexfile           = ,
    //.apsel                      = 0,
};

const target_family_descriptor_t *g_target_family = &g_target_family_mm32;
