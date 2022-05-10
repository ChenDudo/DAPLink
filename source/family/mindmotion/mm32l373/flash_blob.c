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

static const uint32_t mm32l373_flash_code[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x49784879, 0x49796041, 0x47706041, 0x4A754876, 0x49766042, 0x60826041, 0x21006081, 0x68C16001,
    0x43112214, 0x69C060C1, 0xD4060740, 0x49704871, 0x21066001, 0x49706041, 0x20006081, 0x486A4770,
    0x22806901, 0x61014311, 0x15826901, 0x61014391, 0x47702000, 0x2101B530, 0x06C94863, 0x68C16141,
    0x43212414, 0x690160C1, 0x43292504, 0x69016101, 0x43112240, 0x49616101, 0xE0004A5E, 0x68C36011,
    0xD1FB07DB, 0x43A96901, 0x68C16101, 0xD0044221, 0x432168C1, 0x200160C1, 0x2000BD30, 0xB530BD30,
    0x61484951, 0x231468C8, 0x60C84318, 0x6088484D, 0x6088484E, 0x24206908, 0x61084320, 0x22406908,
    0x61084310, 0x4A4B484D, 0x6010E000, 0x07ED68CD, 0x6908D1FB, 0x610843A0, 0x401868C8, 0x68C8D003,
    0x60C84318, 0xBD302001, 0xF7FFB530, 0x4D44FF89, 0xF7FF4628, 0x493CFFD4, 0x4A426908, 0x61084010,
    0x24106908, 0x61084320, 0x8028483F, 0x4A39483B, 0x6010E000, 0x07DB68CB, 0x6908D1FB, 0x610843A0,
    0x221468C8, 0xD0034010, 0x431068C8, 0x200160C8, 0xB500BD30, 0xFFD8F7FF, 0xFF8CF7FF, 0xF7FF4833,
    0x2000FFAE, 0xB530BD00, 0x68CA4927, 0x431A2314, 0x690A60CA, 0x43222402, 0x6148610A, 0x22406908,
    0x61084310, 0x4A234825, 0x6010E000, 0x07ED68CD, 0x6908D1FB, 0x610843A0, 0x401868C8, 0x68C8D003,
    0x60C84318, 0xBD302001, 0x47702001, 0x4D16B5F0, 0x08491C49, 0x004968EB, 0x43232404, 0x271460EB,
    0xE01A4C16, 0x2601692B, 0x612B4333, 0x80038813, 0xE0004B10, 0x68EE601C, 0xD1FB07F6, 0x085B692B,
    0x612B005B, 0x423B68EB, 0x68E8D004, 0x60E84338, 0xBDF02001, 0x1E891C80, 0x29001C92, 0x2000D1E2,
    0x0000BDF0, 0x45670123, 0x40022000, 0xCDEF89AB, 0x00005555, 0x40003000, 0x00000FFF, 0x0000AAAA,
    0x1FFFF800, 0x00001FEF, 0x00005AA5, 0x1FFE0000, 0x00000000
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
    0x2000005F,  // UnInit
    0x20000153,  // EraseChip
    0x20000167,  // EraseSector
    0x200001AD,  // ProgramPage
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
    sizeof(mm32l373_flash_code),  // prog_blob size
    (uint32_t *)(mm32l373_flash_code),  // address of prog_blob
    0x00000400,  // ram_to_flash_bytes_to_be_written
};
