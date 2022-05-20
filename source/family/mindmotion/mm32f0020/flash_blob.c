/* Flash OS Routines (Automagically Generated)
 * Copyright (c) 2009-2019 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

static const uint32_t mm32f0020_flash_code[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x49A648A5, 0x48A66048, 0x47706048, 0x4603B510, 0x4CA248A1, 0x48A26060, 0x489F6060, 0x48A060A0,
    0x200060A0, 0x46206020, 0x241468C0, 0x4C9B4320, 0x462060E0, 0x240469C0, 0x28004020, 0x4899D106,
    0x60204C99, 0x60602006, 0x60A04898, 0xBD102000, 0x48924601, 0x22806900, 0x4A904310, 0x46106110,
    0x15926900, 0x4A8D4390, 0x20006110, 0x20014770, 0x498A06C0, 0x46086148, 0x211468C0, 0x49874308,
    0x460860C8, 0x21046900, 0x49844308, 0x46086108, 0x21406900, 0x49814308, 0xE0026108, 0x49824884,
    0x487E6008, 0x07C068C0, 0x28000FC0, 0x487BD1F6, 0x21046900, 0x49794388, 0x46086108, 0x211468C0,
    0x28004008, 0x4875D006, 0x430868C0, 0x60C84973, 0x47702001, 0xE7FC2000, 0x48704601, 0x68C06141,
    0x43102214, 0x60D04A6D, 0x6090486B, 0x6090486C, 0x69004610, 0x43102220, 0x61104A68, 0x69004610,
    0x43102240, 0x61104A65, 0x4869E002, 0x60104A66, 0x68C04862, 0x0FC007C0, 0xD1F62800, 0x6900485F,
    0x43902220, 0x61104A5D, 0x68C04610, 0x40102214, 0xD0062800, 0x68C04859, 0x4A584310, 0x200160D0,
    0x20004770, 0xB500E7FC, 0xFF5AF7FF, 0xF7FF4859, 0x4852FFC3, 0x49586900, 0x49504008, 0x46086108,
    0x21106900, 0x494D4308, 0x48546108, 0x80084951, 0x484FE002, 0x6008494C, 0x68C04848, 0x0FC007C0,
    0xD1F62800, 0x69004845, 0x43882110, 0x61084943, 0x68C04608, 0x40082114, 0xD0062800, 0x68C0483F,
    0x493E4308, 0x200160C8, 0x2000BD00, 0xB500E7FC, 0xFFC9F7FF, 0xFF5BF7FF, 0xBD002000, 0x48374601,
    0x221468C0, 0x4A354310, 0x461060D0, 0x22026900, 0x4A324310, 0x46106110, 0x69006141, 0x43102240,
    0x61104A2E, 0x4832E002, 0x60104A2F, 0x68C0482B, 0x0FC007C0, 0xD1F62800, 0x69004828, 0x43902202,
    0x61104A26, 0x68C04610, 0x40102214, 0xD0062800, 0x68C04822, 0x4A214310, 0x200160D0, 0x20004770,
    0x4603E7FC, 0x47702001, 0x4603B510, 0x08411C48, 0x481A0049, 0x240468C0, 0x4C184320, 0xE02760E0,
    0x69004816, 0x43202401, 0x61204C14, 0x80188810, 0x4817E002, 0x60204C14, 0x68C04810, 0x0FC007C0,
    0xD1F62800, 0x6900480D, 0x00400840, 0x61204C0B, 0x68C04620, 0x40202414, 0xD0062800, 0x68C04807,
    0x4C064320, 0x200160E0, 0x1C9BBD10, 0x1E891C92, 0xD1D52900, 0xE7F72000, 0x45670123, 0x40022000,
    0xCDEF89AB, 0x00005555, 0x40003000, 0x00000FFF, 0x0000AAAA, 0x1FFFF800, 0x00001FEF, 0x00005AA5,
    0x00000000
};

/**
* List of start and size for each size of flash sector
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address start + size.
*/
static const sector_info_t sectors_info[] = {
    {0x08000000, 0x400},
};

const program_target_t flash_algo = {
    0x2000002D,  // Init
    0x20000071,  // UnInit
    0x200001CF,  // EraseChip
    0x200001DD,  // EraseSector
    0x20000249,  // ProgramPage
    0x0,         // Verify

    // BKPT : start of blob + 1
    // RSB  : address to access global/static data
    // RSP  : stack pointer
    {
        0x20000001,
        0x20000C00,
        0x20001000
    },

    0x20000400,  // mem buffer location
    0x20000000,  // location to write prog_blob in target RAM
    sizeof(mm32f0020_flash_code),  // prog_blob size
    mm32f0020_flash_code,  // address of prog_blob
    0x00000400,  // ram_to_flash_bytes_to_be_written
};
