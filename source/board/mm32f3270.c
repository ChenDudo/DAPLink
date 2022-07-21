/**
 * @file    mm32f3270.c
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
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "target_family.h"
#include "target_board.h"

const board_info_t g_board_info = {
    .info_version       = kBoardInfoVersion,
	.flags				= kEnableUnderResetConnect,
#if defined(MM32LINK_MAX)
    .board_id           = "059",
#elif defined(MM32LINK_MINI)
    .board_id           = "088",
#else
    .board_id           = "001",
#endif
    .family_id          = kMindMotion_FamilyID,
    .daplink_url_name   = "PRODINFOHTM",
#if defined(MM32LINK_MAX)   
    .daplink_drive_name = "MM32-LINK A",
#elif defined(MM32LINK_MINI)
    .daplink_drive_name = "MM32-LINK I",
#else
    .daplink_drive_name = "MM32-LINK",
#endif    
    .daplink_target_url = "https://www.mindmotion.com.cn/support/development_tools/",
    .target_cfg         = &target_device,
    .board_vendor       = "MindMotion",
    .board_name         = "MM32LINK Series",
};
