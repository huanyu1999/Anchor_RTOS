# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded firmware for a **Railroad Active Warning Device — Anchor node**, targeting the **STM32F405VGTx** (Cortex-M4 with FPU). The device uses UWB Two-Way Ranging (TWR) via a DW1000 chip to measure distances to Tag nodes and trigger audio/LED alarms when a tag comes within threshold range. Communication peripherals include CAN bus, W5500 Ethernet, and GNSS.

## Build System

Two parallel build systems exist:
- **CMake + arm-none-eabi-gcc** (primary for command-line builds)
- **Keil MDK-ARM** (`MDK-ARM/Anchor.uvprojx`) for IDE-based development

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
01_Core/App/          - Application logic (FreeRTOS tasks, GNSS, network, alarm state)
01_Core/App/dw1000_application/ - UWB positioning algorithm (TWR anchor role)
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

Defined in `01_Core/App/Src/main.c` and `freertos.c`:
- `task_swoOutput` — SWO debug output (Realtime5, 1024B stack)
- `task_main` — Main process entry (Realtime4, 256B stack), starts all subsystems
- Subtasks for GNSS, W5500/network, DW1000 ranging, alarm state machine, CAN, audio

Key RTOS primitives (`01_Core/App/Inc/os_event.h`):
- `queue_alarm` (`osMessageQueueId_t`) — carries `alarm_event_t` enum values between tasks
- Semaphores: `sema_w5500Int`, `sema_elogLock`, `sema_gnssReceive`

### UWB Ranging (`dw1000_application/`)

- `dw_main.c` — DW1000 init, radio configuration, ranging loop
- `instance_anchor.c` — Anchor-side TWR protocol state machine
- `dw_sort.c` — Processes raw distance measurements, derives min-distance report
- `dw_instance.h` — Shared state: `distance_report[8]`, `group_report[8]`, `range_status`, algorithm pointer

Radio configs are channel-5 presets (850K baud, variable preamble lengths). Active algorithm: `uwbTwr_AnchorAlgorithm`.

### Alarm State Machine

States: `ALARM_IDLE → ALARM_ACTIVE → ALARM_MUTED` (defined in `os_event.h`).

Thresholds in `freertos.c`:
- `TAG_DIS_CHANGE_THRESHOLD = 3000` mm — distance change that re-triggers alarm
- `VOICE_PLAY_MIN_MS = 1700` ms — debounce for audio output

Events that drive transitions are `alarm_event_t` values posted to `queue_alarm`.

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

## Memory 规则

每次对话结束前，主动询问用户是否需要更新 memory。
