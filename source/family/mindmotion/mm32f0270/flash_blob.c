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

static const uint32_t mm32f0270_flash_code[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x49A848A7, 0x48A86048, 0x47706048, 0x4603B510, 0x4CA448A3, 0x48A46060, 0x48A16060, 0x48A260A0,
    0x200060A0, 0x46206020, 0x241468C0, 0x4C9D4320, 0x462060E0, 0x240469C0, 0x28004020, 0x489BD106,
    0x60204C9B, 0x60602006, 0x60A0489A, 0xBD102000, 0x48944601, 0x22806900, 0x4A924310, 0x46106110,
    0x15926900, 0x4A8F4390, 0x20006110, 0x20014770, 0x498C06C0, 0x46086148, 0x211468C0, 0x49894308,
    0x460860C8, 0x21046900, 0x49864308, 0x46086108, 0x21406900, 0x49834308, 0xE0026108, 0x49844886,
    0x48806008, 0x07C068C0, 0x28000FC0, 0x487DD1F6, 0x21046900, 0x497B4388, 0x46086108, 0x211468C0,
    0x28004008, 0x4877D006, 0x430868C0, 0x60C84975, 0x47702001, 0xE7FC2000, 0x48724601, 0x68C06141,
    0x43102214, 0x60D04A6F, 0x6090486D, 0x6090486E, 0x69004610, 0x43102220, 0x61104A6A, 0x69004610,
    0x43102240, 0x61104A67, 0x486BE002, 0x60104A68, 0x68C04864, 0x0FC007C0, 0xD1F62800, 0x69004861,
    0x43902220, 0x61104A5F, 0x68C04610, 0x40102214, 0xD0062800, 0x68C0485B, 0x4A5A4310, 0x200160D0,
    0x20004770, 0xB500E7FC, 0xFF5AF7FF, 0xF7FF485B, 0x4854FFC3, 0x495A6900, 0x49524008, 0x46086108,
    0x21106900, 0x494F4308, 0x48566108, 0x80084953, 0x4851E002, 0x6008494E, 0x68C0484A, 0x0FC007C0,
    0xD1F62800, 0x69004847, 0x43882110, 0x61084945, 0x68C04608, 0x40082114, 0xD0062800, 0x68C04841,
    0x49404308, 0x200160C8, 0x2000BD00, 0xB500E7FC, 0xFFC9F7FF, 0xFF5BF7FF, 0xF7FF4843, 0x2000FF8D,
    0x4601BD00, 0x68C04837, 0x43102214, 0x60D04A35, 0x69004610, 0x43102202, 0x61104A32, 0x61414610,
    0x22406900, 0x4A2F4310, 0xE0026110, 0x4A304832, 0x482C6010, 0x07C068C0, 0x28000FC0, 0x4829D1F6,
    0x22026900, 0x4A274390, 0x46106110, 0x221468C0, 0x28004010, 0x4823D006, 0x431068C0, 0x60D04A21,
    0x47702001, 0xE7FC2000, 0x20014603, 0xB5104770, 0x1C484603, 0x00490841, 0x68C0481A, 0x43202404,
    0x60E04C18, 0x4817E027, 0x24016900, 0x4C154320, 0x88106120, 0xE0028018, 0x4C154817, 0x48116020,
    0x07C068C0, 0x28000FC0, 0x480ED1F6, 0x08406900, 0x4C0C0040, 0x46206120, 0x241468C0, 0x28004020,
    0x4808D006, 0x432068C0, 0x60E04C06, 0xBD102001, 0x1C921C9B, 0x29001E89, 0x2000D1D5, 0x0000E7F7,
    0x45670123, 0x40022000, 0xCDEF89AB, 0x00005555, 0x40003000, 0x00000FFF, 0x0000AAAA, 0x1FFFF800,
    0x00001FEF, 0x00005AA5, 0x1FFE0000, 0x00000000
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
    0x200001E3,  // EraseSector
    0x2000024F,  // ProgramPage
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
    sizeof(mm32f0270_flash_code),  // prog_blob size
    mm32f0270_flash_code,  // address of prog_blob
    0x00000400,  // ram_to_flash_bytes_to_be_written
};
