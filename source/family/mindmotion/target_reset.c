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
#include "gpio.h"

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

#define NVIC_Addr    (0xe000e000)
#define DBG_Addr     (0xe000edf0)

// AP CSW register, base value
#define CSW_VALUE (CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)

#define DCRDR 0xE000EDF8
#define DCRSR 0xE000EDF4
#define DHCSR 0xE000EDF0
#define REGWnR (1 << 16)

/******************************************************************************/
uint8_t read_mcu_Halt(void)
{
	uint32_t tmp;
	uint8_t retry = 5;
	
	do {
		if (!swd_read_word(DBG_HCSR, &tmp)) {
			continue;
		};
		if ((tmp & S_HALT) == 0){
			continue;
		};
		return 1;
	} while(--retry);
	return 0;
}

/*
if (do_abort != 0) {
	//do an abort on stale target, then reset the device
	swd_write_dp(DP_ABORT, DAPABORT);
	PIN_nRESET_OUT(0); osDelay(20);
	PIN_nRESET_OUT(1); osDelay(20);
	do_abort = 0;
}
*/
/******************************************************************************/
int8_t swd_init_debug_mm32(void)
{
	uint32_t tmp;
	uint8_t retries = 100, times = 5;
	//uint8_t specialFlag = 0;

	swd_init();

	do{
		// LINERESET + JTAG2SWD + read IDCODE
		times = 20;
		while (!JTAG2SWD() && --times) {
			if (times == 10) {
				//specialFlag = 1;
				PIN_nRESET_OUT(0);
			}
		};
		if (!JTAG2SWD() && (times == 0)) {
			//specialFlag = 0;
			PIN_nRESET_OUT(1);
			//Power_Off();
			//osDelay(20);
			//Power_Supply();
			return 2;
		}

		// W_DP_ABORT = 0x0000_001e
		times = 5;
		while (!swd_clear_errors() && --times) {
			;
		};

		// W_DP_SELECT = 0x0000_0000; 		Ensure CTRL/STAT selected in DP_BANKSel, if yes pass.
		times = 5;
		while (!swd_write_dp(DP_SELECT, 0) && --times) {
			;
		};

		// R_DP_CTRL/STAT ?= 0xA000_0000;	Read and Check Power Up ACK
		times = 5;
		while (!((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK) && --times)) {
			// W_DP_STRL/STAT = 0x5000_0000;	Power Up Debug & SYSTEM CLOCK
			uint8_t itimes = 5;
			while (!swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ) && --itimes){};
			itimes = 5;
			while (!swd_read_dp(DP_CTRL_STAT, &tmp) && --itimes) {};
		};
		
		// W_DP_SELECT = 0x0000_00F0;		Select AP bank
		times = 5;
		while (!swd_write_dp(DP_SELECT, 0xF0) && --times) {
			continue;
		};

		swd_read_ap(0xFC, &tmp); // M0 0x0477_0021; M3 0x24770011; L0133 0x0477_0031
		swd_read_word(0x40007080, &tmp);

		// 0xE000EDFC = 0x00000001;			Enable halt on reset
		if (!swd_write_word(DBG_EMCR, VC_CORERESET)){
			continue;
		}

		// 0xE000EDF0 = 0xA05F_0003;		Enable Debug and Halt the core
		if (!swd_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT)) {
			continue;
		}

		// 0xE000EDF0 ?= 0x00020000;			Read CPU is/not halt
		if (!swd_read_word(DBG_HCSR, &tmp)) {
			continue;
		};
		if (!(tmp & S_HALT)) {
			continue;
		};

		//    // Disable halt on reset
		//    if (!swd_write_word(DBG_EMCR, 0)){
		//
		//    }
	} while (--retries);

	return 0;
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
	bool firstRun = false;
	
	targetPower = (targetVDD > 2000) ? true : false;
	firstRun = (targetPower & !firstInDetect) ? true : false;
	if (targetVDD >= 3300){
		targetCurrent = (uint16_t)((5000 - targetVDD) / 10);
	}
		
	if (targetPower) {
		if (!firstInDetect){
			firstInDetect = true;
		}
	}
	else {
		firstInDetect = false;
	}
	
	
	if (firstRun){
		{beepMode = mode2; beepCount = 5;}
		//PIN_nRESET_OUT(0);
		//osDelay(20);
		//PIN_nRESET_OUT(1);
        //if (!nRstDetect())
        //    return -1;
        //if (!swd_init_debug_mm32())
        //    return -2;
		
    }
	return 0;
}
