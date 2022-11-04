/**
 * @file    mm32link_board.c
 * @brief   board information for MM32
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS
 " BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "target_family.h"
#include "target_board.h"
#include "swd_host.h"

// typedef struct {
//     uint32_t ID;
//     char *Name;
// } MM32_CPU_LIST;

// char* CPUID;

/******************************************************************************/
// char* swd_read_cpu_id(uint32_t readvalue)
// {
// 	char *p;
// 	const MM32_CPU_LIST list[] = {
// 		{0xCC4350D1, "MM32F0010"},	// SWID: 0x0BB11477
// 		{0x4C50F800, "MM32F0020"},	// SWID: 0x0BB11477
// 		{0X4C50E800, "MM32F0040"},	// SWID: 0x0BB11477, same as F0140
// 		{0xCC5680C7, "MM32F0130"},	// SWID: 0x0BB11477
// 		{0X4C50E800, "MM32F0140"},	// SWID: 0x0BB11477		
// 		{0xCC5680F1, "MM32F0270"},	// SWID: 0x0BB11477
// 		{0xCC9AA0E7, "MM32F3270"},	// SWID: 0x2BA01477
// 		{0x4C4D1000, "MM32L0130"},	// SWID: 0x0BB11477
// 		{0xCC4460B1, "MZ309"},		// SWID: 0x0BB11477, same as F003
// 		//{0x0000CC88, "MT304"},		// SWID: 0x2BA01477, 0xCC88xxx3
// 	};
	
// 	for (uint8_t i = 0; i < 10; i ++) {
// 		if (list[i].ID == readvalue) {
// 			p = list[i].Name;
// 			break;
// 		}
// 		else if (((int)(readvalue) >> 16) == 0xCC88){
// 			p = "MT304";
// 			break;
// 		}
// 		else {
// 			p = "Unknown";
// 			break;
// 		}
// 	}
// 	return p;
// }

/******************************************************************************/
// void prerun_board_config(void)
// {
//     uint32_t tempRead = 0x00;
//     // halt mcu
//     if (target_set_state(HALT) == 0) {
//         target_set_state(RESET_PROGRAM);
//     }
//     // read mcu
//     swd_read_memory(0x40013400, (uint8_t*)&tempRead, 4);
//     CPUID = swd_read_cpu_id(tempRead);
// }

/******************************************************************************/
const board_info_t g_board_info = {
    .info_version       = kBoardInfoVersion,        /*!< Version number of the board info */
    .flags              = kEnableUnderResetConnect|kEnablePageErase,    /*!< Flags from #_board_info_flags */
    .family_id          = kStub_HWReset_FamilyID, //kMindMotion_FamilyID,   	/*!< Use to select or identify target family from defined target family or custom ones */
    .target_cfg         = &target_device,           /*!< Specific chip configuration for the target and enables MSD when non-NULL */
    .board_id           = "059",
    .board_name         = "MM32LINK MAX",          /*!< Board name. Maximum 60 characters including terminal NULL. */
    .daplink_drive_name = "MM32-LINK A",
    //.daplink_target_url = "https://www.mindmotion.com.cn/support/development_tools/debug_and_programming_tools/mm32_link_mini"
};
