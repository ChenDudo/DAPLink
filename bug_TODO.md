# remount 问题

1. cfgram (RAM 配置信息)
   
- (setting.c)
  - begin： 0x2000_bf00
  - size ： 0x8e (144 Byte)
-
``` C
// WARNING - THIS STRUCTURE RESIDES IN RAM STORAGE!
// Be careful with changes:
// -Only add new members to end end of this structure
// -Do not change the order of members in this structure
// -Structure must remain packed so no padding bytes are added
typedef struct __attribute__((__packed__)) cfg_ram {
    uint32_t key;               // Magic key to indicate a valid record
    uint16_t size;              // Offset of the last member from the start

    // Configurable values
    uint8_t hold_in_bl;
    char assert_file_name[64 + 1];
    uint16_t assert_line;
    uint8_t assert_source;

    // Additional debug information on faults
    uint8_t  valid_dumps;
    uint32_t hexdump[ALLOWED_HEXDUMP];  //Alignments checked

    // Disable msd support
    uint8_t disable_msd;

    //Add new entries from here
    uint8_t page_erase_enable;
} cfg_ram_t;
```

2. cfgrom (ROM 配置信息)
   
- (settings_rom.c)
  - begin: 0x0801_fc00
  - size:
    - 0xa(10 Byte)
    - 

```c
typedef struct __attribute__((__packed__)) cfg_setting {
    uint32_t key;               // Magic key to indicate a valid record
    uint16_t size;              // Size of cfg_setting_t

    // Configurable values
    uint8_t auto_rst;
    uint8_t automation_allowed;
    uint8_t overflow_detect;
    uint8_t detect_incompatible_target;

    // Add new members here

} cfg_setting_t;
```
