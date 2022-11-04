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
    0x49764877, 0x49776041, 0x47706041, 0x4a734874, 0x49746042, 0x60826041, 0x21006081, 0x68c16001,
    0x43112214, 0x69c060c1, 0xd4060740, 0x496e486f, 0x21066001, 0x496e6041, 0x20006081, 0x48684770,
    0x22806901, 0x61014311, 0x15826901, 0x61014391, 0x47702000, 0x2101b530, 0x06c94861, 0x68c16141,
    0x43212414, 0x690160c1, 0x43292504, 0x69016101, 0x43112240, 0x495f6101, 0xe0004a5c, 0x68c36011,
    0xd1fb07db, 0x43a96901, 0x68c16101, 0xd0044221, 0x432168c1, 0x200160c1, 0x2000bd30, 0xb530bd30,
    0x6148494f, 0x231468c8, 0x60c84318, 0x6088484b, 0x6088484c, 0x24206908, 0x61084320, 0x22406908,
    0x61084310, 0x4a49484b, 0x6010e000, 0x07ed68cd, 0x6908d1fb, 0x610843a0, 0x401868c8, 0x68c8d003,
    0x60c84318, 0xbd302001, 0xf7ffb530, 0x4d42ff89, 0xf7ff4628, 0x493affd4, 0x4a406908, 0x61084010,
    0x24106908, 0x61084320, 0x8028483d, 0x4a374839, 0x6010e000, 0x07db68cb, 0x6908d1fb, 0x610843a0,
    0x221468c8, 0xd0034010, 0x431068c8, 0x200160c8, 0xb500bd30, 0xffd8f7ff, 0xff8cf7ff, 0xbd002000,
    0x4927b530, 0x231468ca, 0x60ca431a, 0x2402690a, 0x610a4322, 0x69086148, 0x43102240, 0x48256108,
    0xe0004a22, 0x68cd6010, 0xd1fb07ed, 0x43a06908, 0x68c86108, 0xd0034018, 0x431868c8, 0x200160c8,
    0x2001bd30, 0xb5f04770, 0x1c494d15, 0x68eb0849, 0x24040049, 0x60eb4323, 0x4c162714, 0x692be01a,
    0x43332601, 0x8813612b, 0x4b108003, 0x601ce000, 0x07f668ee, 0x692bd1fb, 0x005b085b, 0x68eb612b,
    0xd004423b, 0x433868e8, 0x200160e8, 0x1c80bdf0, 0x1c921e89, 0xd1e22900, 0xbdf02000, 0x45670123,
    0x40022000, 0xcdef89ab, 0x00005555, 0x40003000, 0x00000fff, 0x0000aaaa, 0x1ffff800, 0x00001fef,
    0x00005aa5, 0x00000000
};

// Start address of flash
static const uint32_t flash_start = 0x08000000;
// Size of flash
static const uint32_t flash_size = 0x00080000;

/**
* List of start and size for each size of flash sector - even indexes are start, odd are size
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address flash_start + flash_size.
*/
static const sector_info_t sectors_info[] = {
    0x08000000, 0x00000400,
};

static const program_target_t flash = {
    0x2000002d, // Init
    0x2000005f, // UnInit
    0x20000153, // EraseChip
    0x20000161, // EraseSector
    0x200001a7, // ProgramPage
    0x0, //0x12000001f, // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x20000224,
        0x20001000
    },

    0x20000000 + 0x00000A00,    // mem buffer location
    0x20000000,                 // location to write prog_blob in target RAM
    sizeof(_flash_prog_blob),     // prog_blob size
    (uint32_t *)_flash_prog_blob, // address of prog_blob
    0x00000400         // ram_to_flash_bytes_to_be_written
};
