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

/******************************************************************************/
void prerun_board_config(void)
{
}

const board_info_t g_board_info = {
    .info_version       = kBoardInfoVersion,        /*!< Version number of the board info */
    //.flags              = kEnableUnderResetConnect|kEnablePageErase,    /*!< Flags from #_board_info_flags */
    .prerun_board_config = prerun_board_config,
    .family_id          = kStub_HWReset_FamilyID,     //kMindMotion_FamilyID,   	/*!< Use to select or identify target family from defined target family or custom ones */
    .target_cfg         = &target_device,           /*!< Specific chip configuration for the target and enables MSD when non-NULL */
    .board_id           = "088",
    .board_vendor       = "MindMotion",
    .board_name         = "MM32LINK MINI",          /*!< Board name. Maximum 60 characters including terminal NULL. */
    .daplink_drive_name = "MM32-LINK I",
    .daplink_target_url = "https://www.mindmotion.com.cn/support/development_tools/debug_and_programming_tools/mm32_link_mini"
};
