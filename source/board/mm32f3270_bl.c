/**
 * @file    mm32f3270_bl.c
 * @brief   board ID and meta-data for the hardware interface circuit (HIC) based on STM32F103XB
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

#include "target_config.h"
#include "target_board.h"
#include "target_family.h"

/**
* List of start and size for each size of flash sector
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address start + size.
*/
static const sector_info_t sectors_info[] = {
    {0x08000000 + KB(48), 0x400},
 };

// mm32f3270 target information
target_cfg_t target_device = {
    .version                    = kTargetConfigVersion,
    .sectors_info               = sectors_info,
    .sector_info_length         = (sizeof(sectors_info))/(sizeof(sector_info_t)),
    .flash_regions[0].start     = 0x08000000 + KB(48),
    .flash_regions[0].end       = 0x08000000 + KB(128),
    .flash_regions[0].flags     = kRegionIsDefault,
    .ram_regions[0].start       = 0x20000000,
    .ram_regions[0].end         = 0x2000C000,
    /* .flash_algo not needed for bootloader */
};

//bootloader has no family
const target_family_descriptor_t *g_target_family = NULL;

const board_info_t g_board_info = {
    .info_version       = kBoardInfoVersion,
#if defined(MM32LINK_MINI)
    .board_id           = "088",
#elif defined(MM32LINK_MAX)
    .board_id           = "059",
#else
    .board_id           = "001",
#endif
    .daplink_url_name   = "MM32_DBTHTM",
    .daplink_drive_name = "BootLoader",
    .daplink_target_url = "https://www.mindmotion.com.cn/support/development_tools/",
    .target_cfg         = &target_device,
};
