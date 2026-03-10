
# 设置cmake的最小需求版本
cmake_minimum_required(VERSION 4.0)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 设置C语言标注为C99
set(CMAKE_C_STANDARD 99)

# 显示详细编译命令
set(CMAKE_VERBOSE_MAKEFILE ON)

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

# 设置输出的可执行文件名称
set(EXECUTABLE ${PROJECT_NAME}.elf)

# 收集启动文件
set (STARTUP_FILE
    startup4gcc/startup_stm32f405xx.s
)


# 收集源文件
file(GLOB_RECURSE 01_CORE_SRC_FILES
    01_Core/App/*.c
    01_Core/Board/*.c
    01_Core/Com/*.c
    01_Core/Dev/*.c
    01_Core/Drv/*.c
    01_Core/DW1000/*.c
    # 01_Core/W5500/*.c
)

file(GLOB 02_VENDOR_SRC_FILES
    02_Vendor_Lib/*.c
    02_Vendor_Lib/STM32F4xx_HAL_Driver/Src/*.c
)

file(GLOB 03_MIDDLEWARE_RTOS_SRC_FILES
    03_Middlewares/Third_Party/FreeRTOS/Source/*.c
    03_Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c
    03_Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/*.c
    03_Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c
)

file(GLOB_RECURSE 04_MIDDLEWARE_OTHER_SRC_FILES
    03_Middlewares/Third_Party/FatFS/Src/*.c
    # 03_Middlewares/STM32_USB_Device_Library/Class/MSC/Src/*.c
    # 03_Middlewares/STM32_USB_Device_Library/Core/Src/*.c
)

# file(GLOB_RECURSE EASY_LOGGER_SRC_FILES
#     01_Core/Com/easylogger/*.c
# )

# file(GLOB_RECURSE W5500_DRV_FILES
#     01_Core/W5500/*.c
# )

# set_source_files_properties( 
#     ${02_VENDOR_SRC_FILES}
#     ${EASY_LOGGER_SRC_FILES}
#     ${W5500_DRV_FILES}

#     PROPERTIES
#         COMPILE_OPTIONS  
#             -Wno-unused-parameter
#             -Wno-unused-variable
#             -Wno-unused-function
# )

# 添加可执行目标源文件
add_executable(${EXECUTABLE} 
    ${STARTUP_FILE} 
    ${01_CORE_SRC_FILES} 
    ${02_VENDOR_SRC_FILES} 
    ${03_MIDDLEWARE_RTOS_SRC_FILES} 
    ${04_MIDDLEWARE_OTHER_SRC_FILES}
)

##########################################################################制作一个elog 库##########################################################################
set(ELOG_SRC
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/src/elog.c
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/src/elog_utils.c
    ${CMAKE_CURRENT_LIST_DIR}/01_Core/Com/easylogger/src/elog_async.c
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

target_compile_options(elog 
    PUBLIC
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -Wall
    -Wextra 
    -Wno-pedantic
    -Wno-unused-function
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

target_compile_options(elog 
    PUBLIC
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -Wall
    -Wextra 
    -Wno-pedantic
    -Wno-unused-function
)

# 设置宏定义编译选项
target_compile_definitions(${EXECUTABLE} PRIVATE
    -DUSE_HAL_DRIVER
    -DSTM32F405xx
)

# 设置头文件目录
target_include_directories(${EXECUTABLE} PRIVATE
    01_Core/App/dw1000_application 
    01_Core/App/Inc 
    01_Core/Board/Inc 
    01_Core/Com/compiler 
    01_Core/Com/kalmanfilter 
    01_Core/Com/min_heap 
    01_Core/Com/multi_timer 
    01_Core/Com/multi_button 
    # 01_Core/Com/easylogger/inc 
    # 01_Core/Com/easylogger/plugins/file
    01_Core/Com/uthash
    01_Core/Dev/Inc 
    01_Core/Drv/Inc 
    01_Core/DW1000/decadriver 
    01_Core/DW1000/platform 
    # 01_Core/W5500/Ethernet 
    # 01_Core/W5500/Ethernet/W5500 
    # 01_Core/W5500/Internet/DHCP 
    02_Vendor_Lib 
    02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc 
    02_Vendor_Lib/STM32F4xx_HAL_Driver/Inc/Legacy 
    02_Vendor_Lib/CMSIS/Core/Include 
    02_Vendor_Lib/CMSIS/Device/ST/STM32F4xx/Include 
    02_Vendor_Lib/CMSIS/Include 
    03_Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F 
    03_Middlewares/Third_Party/FreeRTOS/Source/include 
    03_Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 
    03_Middlewares/Third_Party/FatFS/Src 
    03_Middlewares/STM32_USB_Device_Library/Class/MSC/Inc
    03_Middlewares/STM32_USB_Device_Library/Core/Inc
)

# 设置编译选项
target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -g
    -O2
    -Wall
    -Wextra 
    -Wpedantic
    -Wunused-function
    $<$<CONFIG:Debug>:-Og>
)

# 设置链接选项
target_link_options(${EXECUTABLE} PRIVATE
    -T${CMAKE_SOURCE_DIR}/STM32F405XX_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -lc
    -lm
    # -lnosys
    -Wl,-Map=${PROJECT_NAME}.map,--cref,--gc-sections,--print-memory-usage
)

# 在此链接上方两个库
target_link_libraries(${EXECUTABLE} elog w5500)

# Print executable size，打印可执行文件大小
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND arm-none-eabi-size ${EXECUTABLE}
)

# Create hex file，生成hex和bin文件
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
    COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin
)

# Improve clean target，清除target时顺带清除bin hex map文件
set_target_properties(${EXECUTABLE} PROPERTIES ADDITIONAL_CLEAN_FILES
        "${PROJECT_NAME}.bin;${PROJECT_NAME}.hex;${PROJECT_NAME}.map")
