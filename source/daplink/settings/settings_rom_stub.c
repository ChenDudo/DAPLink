/**
 * @file    settings_rom_stub.c
 * @brief   Implementation of settings.h
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

#include "settings.h"

// WARNING - THIS STRUCTURE RESIDES IN NON-VOLATILE STORAGE!
// Be careful with changes:
// -Only add new members to end end of this structure
// -Do not change the order of members in this structure
// -Structure must remain packed so no padding bytes are added
typedef struct __attribute__((__packed__)) cfg_setting {
    uint32_t key;               // Magic key to indicate a valid record
    uint16_t size;              // Size of cfg_setting_t

    // Configurable values
    uint8_t auto_rst;
    uint8_t automation_allowed;
    uint8_t overflow_detect;
    uint8_t detect_incompatible_target;

    // Add new members here
	uint8_t myOption; //bit 0: 3.3/5V; bit 1: beep off/on	
} cfg_setting_t;

#if defined(__CC_ARM)
static volatile cfg_setting_t config_rom __attribute__((section("cfgrom"),zero_init));
#else
static volatile cfg_setting_t config_rom __attribute__((section("cfgrom")));
#endif

void config_rom_init()
{
    // Do nothing
}

void config_set_auto_rst(bool on)
{
    // Do nothing
}

void config_set_automation_allowed(bool on)
{
    // Do nothing
}

void config_set_overflow_detect(bool on)
{
    // Do nothing
}

void config_set_detect_incompatible_target(bool on)
{
    // Do nothing
}

bool config_get_auto_rst()
{
    //return false;
	return config_rom.auto_rst;
}

bool config_get_automation_allowed()
{
    //return true;
	return config_rom.automation_allowed;
}

bool config_get_overflow_detect()
{
    //return false;
	return config_rom.overflow_detect;
}

bool config_get_detect_incompatible_target()
{
    //return false;
	return config_rom.detect_incompatible_target;
}

// TODO chendo new add
void config_set_5v_output(bool on)
{
	// Do nothing
}

bool config_get_5v_output(void)
{
	return config_rom.myOption & 0x01;
}


void config_set_beep_en(bool on)
{
	// Do nothing
}

bool config_get_beep_en(void)
{
	return config_rom.myOption & 0x02;
}
