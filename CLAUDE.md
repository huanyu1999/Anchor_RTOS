# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded firmware for a **Railroad Active Warning Device — Anchor node**, targeting the **STM32F405VGTx** (Cortex-M4 with FPU). The device uses UWB Two-Way Ranging (TWR) via a DW1000 chip to measure distances to Tag nodes and trigger audio/LED alarms when a tag comes within threshold range. Communication peripherals include CAN bus, W5500 Ethernet, and GNSS.

## Build System

**CMake + arm-none-eabi-gcc** is the only build system (the Keil MDK-ARM project has been removed from the repo).

### CMake Build Commands

```bash
# Configure (from project root)
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake

# Build
cmake --build build

# Clean build
cmake --build build --target clean
```

The toolchain (`toolchain.cmake`) auto-discovers `arm-none-eabi-gcc` via `where`/`which`. Build outputs go to `build/`: `Anchor.elf`, `Anchor.hex`, `Anchor.bin`, `Anchor.map`.

### Compiler Flags (Cortex-M4)
`-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -std=c99`

## Architecture

### Layer Structure

```
01_Core/App/          - Application logic (FreeRTOS tasks, GNSS, network, alarm state,
                        UWB TWR anchor algorithm: dw_main.c / dw_instance_anchor.c / dw_sort.c)
01_Core/Board/        - Board-level abstraction (GPIO, DW1000 board init)
01_Core/Dev/          - Device drivers (CAN, button, GNSS, JQ8400 audio)
01_Core/Drv/          - Low-level peripheral drivers (SPI, I2C, UART, timers)
01_Core/DW1000/       - Decawave DW1000 vendor driver + platform port
01_Core/Com/          - Third-party utilities (easylogger, kalman filter, uthash)
01_Core/W5500/        - W5500 Ethernet driver (built as static lib libw5500.a)
02_Vendor_Lib/        - STM32F4 HAL + CMSIS
03_Middlewares/       - FreeRTOS kernel, FatFS, USB MSC
```

### FreeRTOS Task Map

主任务定义在 `01_Core/App/Src/freertos.c`，优先级从高到低：

| 任务 | 优先级 | 栈 | 职责 |
|---|---|---|---|
| `task_uwb` | ISR | 2048 | DW IRQ → process_deca_irq → 回调内直驱 TWR 状态机（onEvent → dwt_starttx/rxenable）；A0 兼任 A2A 周期触发（信号量超时兜底） |
| `task_minHeapManage` | Realtime5 | 1024 | 距离 hash+minheap 插入/更新/purge |
| `task_anchorDisHandling` | Realtime4 | 1280 | 本/对侧距离融合，LED 控制，post queue_alarm |
| `task_getMinDis` | Realtime4 | 1024 | TIM2(20ms) 周期取堆顶，post queue_minimalDis |
| `task_eventHandler` | Realtime3 | 1024 | 分级报警状态机，JQ8400 语音，CAN 发送 |
| `task_tagDisMonitor` | Normal | 1024 | 1500ms 周期检测距离跳变，触发重播 |
| `task_gnssSyncTime` | Normal | 768 | GNSS 时间同步 |
| `task_eth` | Normal | 1024 | W5500 以太网收发（sema_w5500Int 唤醒） |
| `task_rtosMonitor` | Low | 1280 | 10s 周期输出栈水位和运行时统计 |
| `task_swoOutput` | Realtime5 | 1024 | SWO 调试输出（定义在 main.c） |
| `task_main` | Realtime4 | 256 | 系统启动入口（定义在 main.c） |

**设计原则**：task_uwb 独占 ISR 最高级（TWR 状态机在其回调上下文内直驱，Trek1000 同款架构），任何其他任务不得与其同级，保护 TWR 延时发送时间窗口不被抢占。

Key RTOS primitives (`01_Core/App/Inc/os_event.h`):
- `queue_alarm` (`osMessageQueueId_t`) — carries `alarm_event_t` enum values between tasks
- `queue_minimalDis` — 本侧最小距离，task_getMinDis → task_anchorDisHandling
- `queue_processDis` — 单次 TWR 成功后的距离，task_uwb（回调内） → task_minHeapManage
- Semaphores: `sema_uwbInt`（DW IRQ）, `sema_tagDistClear`（TIM2 20ms）, `sema_w5500Int`, `sema_elogLock`, `sema_gnssReceive`

### UWB Ranging (`01_Core/App/Src/`)

- `dw_main.c` — DW1000 init, radio configuration, ranging loop
- `dw_instance_anchor.c` — Anchor-side TWR protocol state machine（T2A + A2A，回调直驱）；`ANCH_FRAME_TRACE_ENABLE` 帧级收发调试日志开关
- `dw_sort.c` — Processes raw distance measurements, derives min-distance report
- `dw_instance.h` — `instance_data_t` 单例（经 `instance_get()` 访问，TWR 内部状态全部收入其中，参照 TREK1000）；对外上报全局变量：`distance_report[8]`, `group_report[8]`, `range_status`

Radio configs are channel-5 presets (850K baud, variable preamble lengths). Active algorithm: `uwbTwr_AnchorAlgorithm`.

### Alarm State Machine

States（`os_event.h`）：`ALARM_LEVEL_0`（无报警）→ `ALARM_LEVEL_1/2/3`（分级）→ `ALARM_MUTED`（静音）

距离门限（`os_event.h`）：
- `ALARM_DIST_FAR = 50000` mm — 50m，Level 1 触发
- `ALARM_DIST_NEAR = 20000` mm — 20m，Level 2 触发
- `ALARM_DIST_DANGER = 10000` mm — 10m，Level 3 触发（同时开蜂鸣器）

语音播报（`freertos.c`）：
- `TAG_DIS_CHANGE_THRESHOLD = 3000` mm — 静音后距离减小超过此值重新触发
- 语音间隔：L1=3600ms / L2=2400ms / L3=800ms（须 ≥ 单条语音实际播放时长）
- <10m 紧急场景只播距离，不播 Tag ID
- JQ8400 UART3 常驻初始化，停播发 Stop 命令，不做 DeInit

Events 通过 `queue_alarm` 以 `alarm_event_t` 传递给 `task_eventHandler`。

### Key Constants / IDs

Device identity pins select `anc_id` at runtime. `group_id` and `tag_id` are stored alongside ranging results.

## Logging

Uses **EasyLogger** (`01_Core/Com/easylogger/`), built as static lib `libelog.a`. SWO output is the primary debug channel (`swo_init()` in `main.c`). Log level and async mode configured in `elog_cfg.h`.

## Hardware Target

| Item | Value |
|------|-------|
| MCU | STM32F405VGTx |
| Flash | 1 MB @ 0x08000000 |
| SRAM | 128 KB @ 0x20000000 + 64 KB CCM |
| Linker script | `STM32F405XX_FLASH.ld` |
| Startup | `startup4gcc/startup_stm32f405xx.s` |
| FreeRTOS tick | 1000 Hz |
| FreeRTOS heap | 4 KB (`heap_4`) |

## TWR 状态机参考
完整规格见 `docs/TREK1000_TWR_STATE_MACHINE_REFERENCE.md`，需要时再读取。
TREK1000 重构（4 阶段）的进度与交接以 `docs/trek1000_refactor_plan.md` 为唯一事实来源。
