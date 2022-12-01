# MM32-LINK Series
## Release Note
----------
# **Revision: 221130**
- New add Offline Download (For MM32F3270)
- `Details.txt` decrease useless info
- Fix up:
  - Reset bug: mm32f0020_BOOT
  - Beep alarm bug

# **Revision: 220930**
- New add Target Voltage/Current Detect: 
  - `targetVDD`: Supply Voltage
  - `targetVCC`: Target ref
  - `targetCurrent`: Supply current
- New add `MM32_V2 CMSIS-DAP` Debugger
  - supported WinUSB Protocol (Win7 need driver)
  - Faster than `MM32_V1` via HID
- Fix up MINI/MAX Upgrade Firmware Package mixed bugs: 
  - Add MINI/MAX's ID division:
    - MINI keep remain `DAPLINK_HIC_ID=0x4D4D3270`
    - new MAX must change to `DAPLINK_HIC_ID=0x4D4D0059`
----------
# **Revision: 220729**
- New Serial Num (Unique ID) Format:
  ```
  Unique_ID[19] = Board_ID[3] + chip_UID[16]
  ```
- New Target Power configure: 
  | Power |  MSC Configure File| Note|
  |--|--|--|
  | Power off|"VT_OFF.CFG"| Target Power OFF |
  | 3.3V output | "VT_3V3.CFG"| Target Power 3.3V |
  | 5V output |"VT_5V.CFG"| Target Power 5V |
- New BEEP configure:
  - MSC Configure File
    - "BEEP_ON.CFG": Open Beep
    - "BEEP_OFF.CFG"
  - BEEP when USB connected / Debug connected
- Fix MM32F0020 debug exception
  - SWCLK Idle state Defalut = Low 

# **Revision: 220715**

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

- Fix up some bugs
  - MSC `.bin` file Supported
  - MSC Storage fix: Available Size of U-disk changed to 16 MB


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

