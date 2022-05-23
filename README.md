# MM32-LINK MAX 
----------
**v0.90 内测版 发布说明**

## 硬件示意图
![MAX](./docs/images/正视图(1).jpg)

## MAIN FUNCTION

| Func | Details | Note |
|:--:|:--:|:--:|
| HID | MM32-V1 Debug | Done|
| CDC | Virtual com port | Done |
| MSC | Drag-n-drop firmware update | Done |

### HID: 
- CMSIS 兼容的调试通道，支持 SW 调试接口
- Windows 免驱，兼容 Windows/Mac OSX/Linux 所有已知版本
  
### MSC:
- 拖拽式升级固件
- 其他功能待开放

### CDC:
- 虚拟串口，收发波特率、数据长度、校验位、停止位可配置

## SUPPORT LISTS

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

---
2022.5.20