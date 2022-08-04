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
#include "debug_cm.h"

#include "DAP_config.h"
#include "hal_gpio.h"
#include "IO_Config.h"
#include "beep.h"

// static void target_before_init_debug(void)
// {
//     // any target specific sequences needed before attaching to the DAP across JTAG or SWD 
//     swd_write_word((uint32_t)&SCB->AIRCR, ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk));
// }

// uint8_t readRstInput;
 static void prerun_target_config(void)
 {
     /* In some case the CPU will enter "cannot debug" state (low power, SWD pin mux changed, etc.). 
 	Doing a hardware reset will clear those states (probably, depends on app). 
 	Also, if the external flash's data is not a valid bootable image, DAPLink cannot attached to target. 
 	A hardware reset will increase the chance to connect in this situation. */

 	GPIO_ResetBits(nRESET_PIN_PORT, nRESET_PIN);
 	osDelay(4);

 	GPIO_SetBits(nRESET_PIN_PORT, nRESET_PIN);
 	osDelay(2);
 }

/*
static void swd_set_target_reset(uint8_t asserted)
{
	if (asserted)
		swd_write_word((uint32_t)&SCB->AIRCR, ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk));
}

static uint8_t target_unlock_sequence(void)
{
    // if the device can secure the flash and there is a way to erase all it should be implemented here.
    return 1;
}
*/

static uint8_t target_set_state(target_state_t state)
{
    
    return 1;
}


//static uint8_t security_bits_set(uint32_t addr, uint8_t *data, uint32_t size)
//{
//    /* if there are security bits in the programmable flash region a check should be performed. 
//	This method is used when programming by drag-n-drop and should refuse to program an image requesting to set the device security. 
//	This can be performed with the debug channel if needed. */
//    return 0;
//}


const target_family_descriptor_t g_target_family_mm32 = {
    .family_id                  = kMindMotion_FamilyID,
    .default_reset_type         = kSoftwareReset,	//kHardwareReset,
    .soft_reset_type            = SYSRESETREQ,
    //.target_before_init_debug   = target_before_init_debug,
    .prerun_target_config       = prerun_target_config,
    //.target_unlock_sequence     = target_unlock_sequence,
    //.security_bits_set          = security_bits_set,
    .target_set_state           = target_set_state,
    //.swd_set_target_reset       = swd_set_target_reset,
    //.validate_bin_nvic          = ,
    //.validate_hexfile           = ,
    //.apsel                      = 0,
};

const target_family_descriptor_t *g_target_family = &g_target_family_mm32;

/******************************************************************************/
int8_t swd_init_debug_mm32(void)
{
	uint32_t tmp;
    uint8_t i, retries = 3, timeout = 10;
	int8_t do_abort = 0;
    
    do {
        if (do_abort) {
            swd_write_dp(DP_ABORT, DAPABORT);
            PIN_nRESET_OUT(0); osDelay(2);
            do_abort = 0;
        }
        swd_init();
		
        /* call a target dependant function this function can do several stuff before really initing the debug */
		//target_before_init_debug();

        if (!JTAG2SWD()) {
            do_abort = 1;
            continue;
        }
        if (!swd_clear_errors()) {
            do_abort = 1;
            continue;
        }
        if (!swd_write_dp(DP_SELECT, 0)) {
            do_abort = 1;
            continue;
        }
        // Power up
        if (!swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
            do_abort = 1;
            continue;
        }
        for (i = 0; i < timeout; i++) {
            if (!swd_read_dp(DP_CTRL_STAT, &tmp)) {
                do_abort = 1;
                break;
            }
            if ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK)) {
                // Break from loop if powerup is complete
                break;
            }
        }
        if ((i == timeout) || (do_abort == 1)) {
            // Unable to powerup DP
            do_abort = 1;
            continue;
        }
        if (!swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE)) {
            do_abort = 1;
            continue;
        }

        /* call a target dependant function: some target can enter in a lock state this function can unlock these targets */
        //target_unlock_sequence();
		
        if (!swd_write_dp(DP_SELECT, 0)) {
            do_abort = 1;
            continue;
        }
        return 1;
    } while (--retries > 0);

    return 0;
}

#define NVIC_Addr    (0xe000e000)
#define DBG_Addr     (0xe000edf0)

// AP CSW register, base value
#define CSW_VALUE (CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)

#define DCRDR 0xE000EDF8
#define DCRSR 0xE000EDF4
#define DHCSR 0xE000EDF0
#define REGWnR (1 << 16)

int8_t swd_halt_mm32(void)
{
	uint32_t val;
    uint8_t ap_retries = 5;

    // Enable debug
    while (swd_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN) == 0) {
        if (--ap_retries <= 0)
            return 0;
        // Target is in invalid state?
        //PIN_nRESET_OUT(0); osDelay(2);
        //PIN_nRESET_OUT(1); osDelay(2);
    }

    // Enable halt on reset
    if (!swd_write_word(DBG_EMCR, VC_CORERESET)) {
        return 0;
    }
	
    do {
        if (!swd_read_word(DBG_HCSR, &val)) {
            return 0;
        }
    } while ((val & S_HALT) == 0);

    // Disable halt on reset
    if (!swd_write_word(DBG_EMCR, 0)){
        return 0;
    }

    return 1;
}

/******************************************************************************/
uint8_t nRstDetect(void)
{
    static uint8_t nrst_retries = 5;
    while (!PIN_nRESET_IN() && --nrst_retries) {
        PIN_nRESET_OUT(1); osDelay(2);
    }
    if (!nrst_retries) return 0;
    return 1;
}

/******************************************************************************/
int8_t handleMCU(void)
{
	static bool targetPower = false;
	static bool firstInDetect = false;
	
	targetPower = (targetVDD > 2000) ? true : false;
	firstInDetect = (targetPower & !firstInDetect) ? true : false;
	
	if (firstInDetect){
        if (nRstDetect())
            return -1;
        if (!swd_init_debug_mm32())
            return -2;
        if (!swd_halt_mm32())
            return -3;
    }
	return 0;
}
