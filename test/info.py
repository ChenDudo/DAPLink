#
# DAPLink Interface Firmware
# Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from __future__ import absolute_import

# Name of all projects ready for public release
# and info on the file to be distributed
PROJECT_RELEASE_INFO = [
    # Project Name                                  Legacy      Offset      Extension
    ("mm32link_max_hid_if",                         True,       0x8000,     "bin"       ),
    ("mm32link_max_winusb_if",                      False,      0x8000,     "bin"       ),
    ("mm32link_mini_hid_if",                        True,       0x8000,     "bin"       ),
    ("mm32link_mini_winusb_if",                     True,       0x8000,     "bin"       ),
   # ("mm32link_mm32f3270_ob_if",                    True,       0x8000,     "bin"       ),
]

TEST_RELEASE_INFO = [
]


# Add new HICs here
VENDOR_ID = {
    'Stub': 0,
    'MindMotion': 1
}

def VENDOR_TO_FAMILY(x, y) : return (VENDOR_ID[x] <<8) | y

# All supported configurations
SUPPORTED_CONFIGURATIONS = [
    #   Board ID    Family ID                           Firmware                                    Bootloader          Target
    (   0x059,     VENDOR_TO_FAMILY('Stub', 1),        'mm32link_max_hid_if',                      'mm32link_max_bl',  'mb039_daplink_serial_validation'                  ),
    (   0x000,     VENDOR_TO_FAMILY('Stub', 1),        'mm32link_max_winusb_if',                   'mm32link_max_bl',  'mb039_daplink_serial_validation'                  ),
    (   0x088,     VENDOR_TO_FAMILY('Stub', 1),        'mm32link_mini_hid_if',                     'mm32link_mini_bl', 'mb039_daplink_serial_validation'                  ),
    (   0x088,     VENDOR_TO_FAMILY('Stub', 1),        'mm32link_mini_winusb_if',                  'mm32link_mini_bl', 'mb039_daplink_serial_validation'                  ),
   # (   0x088,     VENDOR_TO_FAMILY('Stub', 1),        'mm32link_mm32f3270_ob_if',                 'mm32link_mini_bl', 'mb039_daplink_serial_validation'                  ),
    # Test projects
]

# Add new HICs here
HIC_STRING_TO_ID = {
    'mm32link': 0x4D4D3270,
}

BOARD_ID_LOCKED_WHEN_ERASED = set([
])

BOARD_ID_SUPPORTING_PAGE_ERASE = set([
])

#Hack until these targets have an image with a valid vector table
TARGET_WITH_BAD_VECTOR_TABLE_LIST = [
]

BOARD_ID_TO_BUILD_TARGET = {config[0]: config[4] for config in SUPPORTED_CONFIGURATIONS}
FIRMWARE_SET = set((config[2] for config in SUPPORTED_CONFIGURATIONS))
TARGET_SET = set((target[4] for target in SUPPORTED_CONFIGURATIONS if target[4] is not None))

TARGET_WITH_COMPILE_API_LIST = [config[4] for config in SUPPORTED_CONFIGURATIONS if config[4] is not None]
