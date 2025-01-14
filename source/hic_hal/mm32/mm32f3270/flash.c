/**
 * @file    flash.c
 * @brief
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
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mm32_device.h"
#include "hal_flash.h"

#include "flash_hal.h"        // FlashOS Structures
#include "target_config.h"    // target_device
#include "util.h"
#include "string.h"
#include "target_board.h"

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    //
    // No special init required
    //
    return (0);
}

uint32_t UnInit(uint32_t fnc)
{
    //
    // No special uninit required
    //
    return (0);
}

uint32_t EraseChip(void)
{
    return (0);
}

uint32_t EraseSector(uint32_t adr)
{
    FLASH_Unlock();
    FLASH_ErasePage(adr);
    FLASH_Lock();
    return (0);  // 0=O.K.
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    uint32_t current_addr = adr; 
	uint32_t len = sz;
	
    FLASH_Unlock();
	
    while(len > 0) {
		if(FLASH_ProgramWord(current_addr, *buf) != FLASH_COMPLETE)
			return 1;
        buf++;
        current_addr += 4;
        len -= 4;
    }
	
    FLASH_Lock();
    return 0; 		// Finished without Errors
}
