/**
 * @file    settings_rom.c
 * @brief   Implementation of settings.h
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2020 Arm Limited, All Rights Reserved
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

#include <string.h>

#include "settings.h"
#include "target_config.h"
#include "compiler.h"
#include "cortex_m.h"
#include "flash_hal.h"

// 'kvld' in hex - key valid
#define CFG_KEY             0x6b766c64
#define SECTOR_BUFFER_SIZE  16

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

// Make sure FORMAT in generate_config.py is updated if size changes
COMPILER_ASSERT(sizeof(cfg_setting_t) == 11);

// Sector buffer must be as big or bigger than settings
COMPILER_ASSERT(sizeof(cfg_setting_t) < SECTOR_BUFFER_SIZE);
// Sector buffer must be a multiple of 4 bytes at least.
// ProgramPage for some interfaces, like the k20dx, require that
// the data is a multiple of 4 bytes, otherwise programming will
// fail.  Assert 8 byte alignement just to be safe.
COMPILER_ASSERT(SECTOR_BUFFER_SIZE % 8 == 0);

// Configuration ROM
#if defined(__CC_ARM)
static volatile cfg_setting_t config_rom __attribute__((section("cfgrom"),zero_init));
#else
static volatile cfg_setting_t config_rom __attribute__((section("cfgrom")));
#endif
// Ram copy of ROM config
static cfg_setting_t config_rom_copy;

// Configuration defaults in flash
static const cfg_setting_t config_default = {
    .auto_rst = 1,
    .automation_allowed = 1,
    .overflow_detect = 1,
    .detect_incompatible_target = 0
};

// Check if the configuration in flash needs to be updated
static bool config_needs_update()
{
    // Update if cfgrom cannot be read (needs to be programmed).
    if (!flash_is_readable((uint32_t)&config_rom, sizeof(config_rom))) {
        return true;
    }

    // Update if the key is invalid
    if (CFG_KEY != config_rom.key) {
        return true;
    }

    // Update if the config key is valid but
    // has a smaller size.
    if (config_rom.size < sizeof(config_rom)) {
        return true;
    }

    // The config is valid and has the right
    // size so it does not need to be updated
    return false;
}

// Reprogram the new settings if flash writing is allowed
static void program_cfg(cfg_setting_t *new_cfg)
{
    uint32_t status;
    uint32_t addr;

    addr = (uint32_t)&config_rom;
    status = flash_erase_sector(addr);
    if (status != 0) {
        return;
    }

    // Buffer for data to flash
    uint8_t write_buffer[SECTOR_BUFFER_SIZE] __ALIGNED(4);

    memset(write_buffer, 0xFF, sizeof(write_buffer));
    memcpy(write_buffer, new_cfg, sizeof(cfg_setting_t));
    status = flash_program_page(addr, sizeof(write_buffer), write_buffer);
    if (status != 0) {
        return;
    }
}

void config_rom_init()
{
    Init(0, 0, 0);

    // Fill in the ram copy with the defaults
    memcpy(&config_rom_copy, &config_default, sizeof(config_rom_copy));

    if (flash_is_readable((uint32_t)&config_rom, sizeof(config_rom))) {
        // Read settings from flash if the key is valid
        if (CFG_KEY == config_rom.key) {
            uint32_t size = MIN(config_rom.size, sizeof(config_rom));
            memcpy(&config_rom_copy, (void *)&config_rom, size);
        }
    }

    // Fill in special values
    config_rom_copy.key = CFG_KEY;
    config_rom_copy.size = sizeof(config_rom);

    // Write settings back to flash if they are out of date
    // Note - program_cfg only programs data in bootloader mode
    if (config_needs_update()) {
        // Program with defaults if none are set
        program_cfg(&config_rom_copy);
    }
}


void config_set_auto_rst(bool on)
{
    config_rom_copy.auto_rst = on;
    program_cfg(&config_rom_copy);
}

void config_set_automation_allowed(bool on)
{
    config_rom_copy.automation_allowed = on;
    program_cfg(&config_rom_copy);
}

void config_set_overflow_detect(bool on)
{
    config_rom_copy.overflow_detect = on;
    program_cfg(&config_rom_copy);
}

void config_set_detect_incompatible_target(bool on)
{
    config_rom_copy.detect_incompatible_target = on;
    program_cfg(&config_rom_copy);
}

bool config_get_auto_rst()
{
    return config_rom_copy.auto_rst;
}

bool config_get_automation_allowed(void)
{
    return config_rom_copy.automation_allowed;
}

bool config_get_overflow_detect()
{
    return config_rom_copy.overflow_detect;
}

bool config_get_detect_incompatible_target()
{
    return config_rom_copy.detect_incompatible_target;
}

// TODO chendo new add
void config_set_5v_output(bool on)
{
	// on = 5V, off = 3.3v
	(on) ? (config_rom_copy.myOption |= 0x01) : (config_rom_copy.myOption &= ~0x01);
	config_set_power_output(true);
	program_cfg(&config_rom_copy);
}

bool config_get_5v_output(void)
{
	return (config_rom.myOption & 0x01);
}

void config_set_power_output(bool on)
{
	(on) ? (config_rom_copy.myOption |= 0x04) : (config_rom_copy.myOption &= ~0x04);
	program_cfg(&config_rom_copy);
}

bool config_get_power_output(void)
{
	return (config_rom.myOption & 0x04);
}

void config_set_beep_en(bool on)
{
	(on) ? (config_rom_copy.myOption |= 0x02) : (config_rom_copy.myOption &= ~0x02);
	program_cfg(&config_rom_copy);
}

bool config_get_beep_en(void)
{
	return (config_rom.myOption & 0x02);
}
