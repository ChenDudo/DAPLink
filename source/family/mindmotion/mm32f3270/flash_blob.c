/* Flash OS Routines (Automagically Generated)
 * Copyright (c) 2009-2015 ARM Limited
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

static const uint32_t _flash_prog_blob[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x49764877, 0x49776041, 0x47706041, 0x4A734874, 0x49746042, 0x60826041, 0x21006081, 0x68C16001,
    0x43112214, 0x69C060C1, 0xD4060740, 0x496E486F, 0x21066001, 0x496E6041, 0x20006081, 0x48684770,
    0x22806901, 0x61014311, 0x15826901, 0x61014391, 0x47702000, 0x2101B530, 0x06C94861, 0x68C16141,
    0x43212414, 0x690160C1, 0x43292504, 0x69016101, 0x43112240, 0x495F6101, 0xE0004A5C, 0x68C36011,
    0xD1FB07DB, 0x43A96901, 0x68C16101, 0xD0044221, 0x432168C1, 0x200160C1, 0x2000BD30, 0xB530BD30,
    0x6148494F, 0x231468C8, 0x60C84318, 0x6088484B, 0x6088484C, 0x24206908, 0x61084320, 0x22406908,
    0x61084310, 0x4A49484B, 0x6010E000, 0x07ED68CD, 0x6908D1FB, 0x610843A0, 0x401868C8, 0x68C8D003,
    0x60C84318, 0xBD302001, 0xF7FFB530, 0x4D42FF89, 0xF7FF4628, 0x493AFFD4, 0x158A6908, 0x61084310,
    0x24106908, 0x61084320, 0x8028483C, 0x4A374839, 0x6010E000, 0x07DB68CB, 0x6908D1FB, 0x610843A0,
    0x221468C8, 0xD0034010, 0x431068C8, 0x200160C8, 0xB500BD30, 0xFFD8F7FF, 0xFF8CF7FF, 0xBD002000,
    0x4927B530, 0x231468CA, 0x60CA431A, 0x2402690A, 0x610A4322, 0x69086148, 0x43102240, 0x48256108,
    0xE0004A22, 0x68CD6010, 0xD1FB07ED, 0x43A06908, 0x68C86108, 0xD0034018, 0x431868C8, 0x200160C8,
    0x2001BD30, 0xB5F04770, 0x1C494D15, 0x68EB0849, 0x24040049, 0x60EB4323, 0x4C162714, 0x692BE01A,
    0x43332601, 0x8813612B, 0x4B108003, 0x601CE000, 0x07F668EE, 0x692BD1FB, 0x005B085B, 0x68EB612B,
    0xD004423B, 0x433868E8, 0x200160E8, 0x1C80BDF0, 0x1C921E89, 0xD1E22900, 0xBDF02000, 0x45670123,
    0x40022000, 0xCDEF89AB, 0x00005555, 0x40003000, 0x00000FFF, 0x0000AAAA, 0x1FFFF800, 0x00005AA5,
    0x00000000
};

// Start address of flash
static const uint32_t flash_start = 0x08000000;
// Size of flash
static const uint32_t flash_size = 0x00004000;

/**
* List of start and size for each size of flash sector - even indexes are start, odd are size
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address flash_start + flash_size.
*/
static const sector_info_t sectors_info[] = {
    0x08000000, 0x400,
};

static const program_target_t flash = {
    0x2000002D, // Init
    0x2000005F, // UnInit
    0x20000153, // EraseChip
    0x20000161, // EraseSector
    0x200001A7, // ProgramPage
    0,          // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x20000624,
        0x20000800
    },

    0x20000224, // mem buffer location: 0x20000000 + 0x00000490
    0x20000000, // location to write prog_blob in target RAM
    sizeof(_flash_prog_blob), // prog_blob size: 490
    _flash_prog_blob, // address of prog_blob: (uint32_t *)_flash_prog_blob
    0x400       // ram_to_flash_bytes_to_be_written
};
