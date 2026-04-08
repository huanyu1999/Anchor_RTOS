
# 设置cmake的最小需求版本
cmake_minimum_required(VERSION 4.0)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 设置C语言标注为C99
set(CMAKE_C_STANDARD 99)

# 显示详细编译命令
set(CMAKE_VERBOSE_MAKEFILE ON)

# 生成 compile_commands.json，供 clangd 代码补全使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_TOOLCHAIN_FILE 
    ${CMAKE_CURRENT_LIST_DIR}/toolchain.cmake
    CACHE FILEPATH "Toolchain file"    
)

# 使用工具链文件
if(NOT CMAKE_TOOLCHAIN_FILE)
    message(FATAL_ERROR "请使用 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake")
endif()

# 配置项目名称以及使用的语言
project(Anchor C CXX ASM)

# 两个可执行目标：分别对应 DW1000 和 DW3000
set(TARGETS Anchor_Dw1000 Anchor_Dw3000)

# 收集启动文件
set(STARTUP_FILE
    startup4gcc/startup_stm32f405xx.s
)

# 收集源文件 —— 自己的代码（App / Board / Dev / Drv，不含 DW 相关 board 文件）
file(GLOB_RECURSE 01_CORE_SRC_FILES
    01_Core/App/*.c
    01_Core/Board/*.c
    01_Core/Dev/*.c
    01_Core/Drv/*.c
)
# 从公共源文件中排除 board_dw*.c，这些按目标分别添加
list(FILTER 01_CORE_SRC_FILES EXCLUDE REGEX "board_dw[0-9]+\\.c$")

# 收集源文件 —— 01_Core 下的第三方代码（Com 组件，不含 easylogger 和 DW 厂家驱动）
# easylogger 已作为独立静态库 libelog.a 编译，此处排除避免重复
file(GLOB_RECURSE 01_CORE_THIRDPARTY_SRC_FILES
    01_Core/Com/*.c
)
list(FILTER 01_CORE_THIRDPARTY_SRC_FILES EXCLUDE REGEX "easylogger/")

# 收集 DW1000 / DW3000 厂家驱动源文件
file(GLOB_RECURSE DW1000_VENDOR_SRC_FILES  01_Core/Dw1000/*.c)
file(GLOB_RECURSE DW3000_VENDOR_SRC_FILES  01_Core/Dw3000/*.c)

file(GLOB 02_VENDOR_SRC_FILES
    02_Vendor_Lib/*.c
    02_Vendor_Lib/STM32F4xx_HAL_Driver/Src/*.c
)

# 主目标中的第三方源文件关闭所有警告（-w）
set_source_files_properties(
    ${01_CORE_THIRDPARTY_SRC_FILES}
    ${DW1000_VENDOR_SRC_FILES}
    ${DW3000_VENDOR_SRC_FILES}
    ${02_VENDOR_SRC_FILES}
    PROPERTIES COMPILE_OPTIONS "-w"
)

# 添加两个可执行目标
add_executable(Anchor_Dw1000.elf
    ${STARTUP_FILE}
    ${01_CORE_SRC_FILES}
    ${01_CORE_THIRDPARTY_SRC_FILES}
    ${DW1000_VENDOR_SRC_FILES}
    ${02_VENDOR_SRC_FILES}
    01_Core/Board/Src/board_dw1000.c
)

add_executable(Anchor_Dw3000.elf
    ${STARTUP_FILE}
    ${01_CORE_SRC_FILES}
    ${01_CORE_THIRDPARTY_SRC_FILES}
    ${DW3000_VENDOR_SRC_FILES}
    ${02_VENDOR_SRC_FILES}
    01_Core/Board/Src/board_dw3000.c
)

##########################################################################制作一个elog 库##########################################################################
set(ELOG_SRC
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/src/elog.c
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/src/elog_utils.c
    # ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/src/elog_async.c
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/port/elog_port.c
)

add_library(elog STATIC ${ELOG_SRC})

target_include_directories(elog 
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/inc
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/plugins/file
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/App/Inc
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Dev/Inc
    ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F 
    ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FreeRTOS/Source/include 
)

target_compile_options(elog PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -w                  # 第三方代码，关闭所有警告
)

##########################################################################制作一个 TinyUSB 库（MSC device over USB OTG FS / DWC2）##########################################################################
set(TINYUSB_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/TinyUsb/src)

add_library(tinyusb STATIC
    ${TINYUSB_SRC_DIR}/tusb.c
    ${TINYUSB_SRC_DIR}/common/tusb_fifo.c
    ${TINYUSB_SRC_DIR}/device/usbd.c
    ${TINYUSB_SRC_DIR}/device/usbd_control.c
    ${TINYUSB_SRC_DIR}/class/msc/msc_device.c
    ${TINYUSB_SRC_DIR}/portable/synopsys/dwc2/dcd_dwc2.c
    ${TINYUSB_SRC_DIR}/portable/synopsys/dwc2/dwc2_common.c
)

target_include_directories(tinyusb
    PUBLIC
        ${TINYUSB_SRC_DIR}                                  # tusb.h, tusb_option.h
        ${TINYUSB_SRC_DIR}/common
        ${TINYUSB_SRC_DIR}/device
        ${TINYUSB_SRC_DIR}/class/msc
        ${TINYUSB_SRC_DIR}/portable/synopsys/dwc2
        
        ${CMAKE_CURRENT_LIST_DIR}/01_Core/App/Inc          # tusb_config.h 由 App/Inc 提供（PUBLIC 可被主目标继承引用）
    PRIVATE
        # TinyUSB 内部需要 STM32F4 HAL 头和 FreeRTOS 头
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Core/Include
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Include
        ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FreeRTOS/Source/include
        ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
)

target_compile_definitions(tinyusb
    PRIVATE
        USE_HAL_DRIVER
        STM32F405xx
)

target_compile_options(tinyusb PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -w                  # 第三方代码，关闭所有警告
)

##########################################################################制作一个w5500驱动库##########################################################################
set(W5500_DRV_SRC
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/W5500/Ethernet/socket.c
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/W5500/Ethernet/W5500/w5500.c
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/W5500/Ethernet/wizchip_conf.c
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/W5500/Internet/DHCP/dhcp.c
)

add_library(w5500 STATIC ${W5500_DRV_SRC})

target_include_directories(w5500 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/W5500/Ethernet
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/W5500/Ethernet/W5500
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/W5500/Internet/DHCP
)

target_compile_options(w5500 PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -w                  # 第三方代码，关闭所有警告
)

##########################################################################制作一个 FreeRTOS 库##########################################################################
set(FREERTOS_DIR ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FreeRTOS/Source)

file(GLOB FREERTOS_SRC
    ${FREERTOS_DIR}/*.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F/*.c
    ${FREERTOS_DIR}/portable/MemMang/*.c
    ${FREERTOS_DIR}/CMSIS_RTOS_V2/*.c
)

add_library(freertos STATIC ${FREERTOS_SRC})

target_include_directories(freertos
    PUBLIC
        ${FREERTOS_DIR}/include
        ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
        ${FREERTOS_DIR}/CMSIS_RTOS_V2
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/01_Core/App/Inc        # FreeRTOSConfig.h
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Core/Include   # cmsis_compiler.h
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Include
)

target_compile_definitions(freertos PRIVATE
    USE_HAL_DRIVER
    STM32F405xx
)

target_compile_options(freertos PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -w                  # 第三方代码，关闭所有警告
)

##########################################################################制作一个 FatFS 库##########################################################################
set(FATFS_DIR ${CMAKE_CURRENT_LIST_DIR}/03_Middlewares/Third_Party/FatFS/Src)

add_library(fatfs STATIC
    ${FATFS_DIR}/ff.c
    ${FATFS_DIR}/diskio.c
)

target_include_directories(fatfs
    PUBLIC
        ${FATFS_DIR}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/01_Core/Dev/Inc        # dev_emmc.h, dev_linkFatFs.h
        ${CMAKE_CURRENT_LIST_DIR}/01_Core/Drv/Inc
        ${CMAKE_CURRENT_LIST_DIR}/01_Core/App/Inc
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_CURRENT_LIST_DIR}/02_Vendor_Lib/CMSIS/Core/Include
)

target_compile_definitions(fatfs PRIVATE
    USE_HAL_DRIVER
    STM32F405xx
)

target_compile_options(fatfs PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -w                  # 第三方代码，关闭所有警告
)

# diskio.c 间接依赖 FreeRTOS 头文件（通过 dev_emmc.h -> cmsis_os.h）
target_link_libraries(fatfs PRIVATE freertos)

# 公共头文件目录（不含 DW 厂家驱动）
set(COMMON_INCLUDE_DIRS
    01_Core/App/Inc
    01_Core/Board/Inc
    01_Core/Com/compiler
    01_Core/Com/kalmanfilter
    01_Core/Com/min_heap
    01_Core/Com/multi_timer
    01_Core/Com/multi_button
    01_Core/Com/uthash
    01_Core/Dev/Inc
    01_Core/Drv/Inc
    02_Vendor_Lib
    02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc
    02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc/Legacy
    02_Vendor_Lib/CMSIS/Core/Include
    02_Vendor_Lib/CMSIS/Device/ST/STM32F4xx/Include
    02_Vendor_Lib/CMSIS/Include
    # W5500、FreeRTOS、FatFS 头文件由各自库的 PUBLIC include 提供
    # TinyUSB 公开头文件（tusb_config.h 已在 App/Inc 中，其余由 tinyusb 库 PUBLIC 继承）
    03_Middlewares/Third_Party/TinyUsb/src
    03_Middlewares/Third_Party/TinyUsb/src/class/msc
    03_Middlewares/Third_Party/TinyUsb/src/device
)

# 公共编译选项
set(COMMON_COMPILE_OPTIONS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -fdiagnostics-color=always
    -g
    -O2
    -Wall
    -Wextra
    -Wno-pedantic       # 避免第三方 PUBLIC 头文件（elog __FUNCTION__、TinyUSB 匿名 union）触发大量噪音
    -Wunused-function
    $<$<CONFIG:Debug>:-Og>
)

# 公共链接选项
set(COMMON_LINK_OPTIONS
    -T${CMAKE_SOURCE_DIR}/STM32F405XX_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -lc
    -lm
    # -lnosys
    -Wl,--cref,--gc-sections,--print-memory-usage
)

# 为两个目标分别配置
foreach(TGT IN LISTS TARGETS)
    set(ELF ${TGT}.elf)

    # 宏定义：HAL + 芯片选择
    target_compile_definitions(${ELF} PRIVATE
        USE_HAL_DRIVER
        STM32F405xx
    )

    # 头文件：公共 + 对应 DW 厂家驱动
    if(TGT STREQUAL "Anchor_Dw1000")
        target_compile_definitions(${ELF} PRIVATE USE_DW1000)
        target_include_directories(${ELF} PRIVATE
            ${COMMON_INCLUDE_DIRS}
            01_Core/Dw1000/decadriver
            01_Core/Dw1000/platform
        )
    else()
        target_compile_definitions(${ELF} PRIVATE USE_DW3000)
        target_include_directories(${ELF} PRIVATE
            ${COMMON_INCLUDE_DIRS}
            01_Core/Dw3000/decadriver
            01_Core/Dw3000/platform
        )
    endif()

    # 编译选项
    target_compile_options(${ELF} PRIVATE ${COMMON_COMPILE_OPTIONS})

    # 链接选项（各自生成 .map 文件）
    target_link_options(${ELF} PRIVATE
        ${COMMON_LINK_OPTIONS}
        -Wl,-Map=${TGT}.map
    )

    # 链接静态库
    target_link_libraries(${ELF} elog w5500 tinyusb freertos fatfs)

    # 打印可执行文件大小
    add_custom_command(TARGET ${ELF}
        POST_BUILD
        COMMAND arm-none-eabi-size ${ELF}
    )

    # 生成 hex 和 bin 文件
    add_custom_command(TARGET ${ELF}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${ELF} ${TGT}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${ELF} ${TGT}.bin
    )

    # 清除 target 时顺带清除 bin hex map 文件
    set_target_properties(${ELF} PROPERTIES ADDITIONAL_CLEAN_FILES
        "${TGT}.bin;${TGT}.hex;${TGT}.map")
endforeach()
