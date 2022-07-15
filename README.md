# MM32-LINK Series

----------

# **Version: 220715**

## Release Note
- Import **MM32-LINK MINI** project
  - added `SWDIO` Dir Control
  - added `nRst` Dir Control
  - ... more
- Supported 3.3V/5V Target Power output
- Supported Beep control
- Serial Num (Unique ID) Format
  ```
  Unique_ID[25] = Board_ID[3] + Version[6] + chip_UID[16]
  ```
  Such as Unique ID = `088`-`220715`-`0ff20f17004c75fd`

  - BoardID 
    - [ ] 059 = MM32-LINK MAX  (MB-059)
    - [ ] 088 = MM32-LINK MINI (MB-088)
    - [ ] ...
- Udisk Size = **512MB**
- Fix up some bugs

---


# **Version: 220520**

This Version is only supported for MM32LINK-MAX.

### MAIN FUNCTION

| NAME | Function | Note |
|:--:|:--:|:--:|
| HID | MM32-V1 Debug | Done|
| CDC | Virtual com port | Done |
| MSC | Drag-n-drop firmware update | Done |

#### Details

- HID

- [x] CMSIS 兼容的调试通道，支持 SW 调试接口
- [x] Windows 免驱，兼容 Windows/Mac OSX/Linux 所有已知版本
  
- MSC

- [x] 拖拽式升级固件
- [ ] 其他功能待开放

- CDC

- [ ] 虚拟串口
  - 可配置收发波特率、数据长度、校验位、停止位


**SUPPORT LISTS**

| Target Chip | Keil(download) |ARM Kernel | Note |
|-------------|:--------------:|-----------|------|
| MM32F0020B1T | Ok | Cortex-M0 | 3310 |
| MM32F0144C6P | Ok | Cortex-M0 | 3290 |
| MM32F3277G9P | Ok | Cortex-M3 | 3270 |
| MM32F0273D7P | Ok | Cortex-M0 | 3260 |
| MM32F0010A1T | Ok | Cortex-M0 | MZ311 |
| MM32F0133C7P | Ok | Cortex-M0 | MZ310 |
| MM32F031C6T6(q1) | Ok | Cortex-M0 | MZ309 |
| MM32SPIN27PF | Ok | Cortex-M0 | MZ308 |
| MM32F103CET6 | Ok | Cortex-M3 | MT307 |
| MM32L073 | Ok | Cortex-M0 | MZ306 |
| MM32L373 | Ok | Cortex-M3 | MT304 |

