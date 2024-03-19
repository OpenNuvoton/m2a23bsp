cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)


set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PATH C:/Keil_v5/ARM/ARMCLANG)
find_program(ARM_CC NAMES armclang  PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_CXX NAMES armclang  PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_ASM NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_AR NAMES armar PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_LINK NAMES armlink PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_FROMELF NAMES fromelf PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)

set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")
set(CMAKE_ASM_COMPILER ${ARM_ASM} CACHE STRING "")
set(CMAKE_LINKER ${ARM_LINK} CACHE STRING "")


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

set(C_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m23 -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -D__MICROLIB")
set(CMAKE_C_FLAGS "-xc -std=c99  ${C_FLAGS}")
set(CMAKE_CXX_FLAGS ${C_FLAGS})
set(CMAKE_ASM_FLAGS "-mcpu=Cortex-M23 -g -mthumb -mfloat-abi=soft --target=arm-arm-none-eabi")
set(CMAKE_EXECUTABLE_SUFFIX ".axf")


project(m2a23bsp)

function(create_project PRJ)
    add_executable(${PRJ} ${ARGN})
    target_link_libraries(${PRJ} system_startup standard_driver)

    target_link_options(${PRJ}
        PUBLIC
        --cpu=Cortex-M23
        --library_type=microlib
        --ro-base 0x00000000
        --rw-base 0x20000000
        --entry Reset_Handler
        --first __Vectors
        --strict
        --map
        --datacompressor=off
        --info=inline
        --entry Reset_Handler
        --summary_stderr
        --info totals
        --map
        --load_addr_map_info
        --xref
        --callgraph
        --symbols
    )

    add_custom_command(TARGET ${PRJ} POST_BUILD
        COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
        VERBATIM
    )
endfunction()



set(BSP_DIR  ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_PATH}/include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/m2a23/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)
include_directories(${BSP_DIR}/Library/NuMaker/common)

# Build library
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/m2a23/Source)
add_subdirectory(${BSP_DIR}/Library/NuMaker/xmodem)


# Build sample code
add_subdirectory(${BSP_DIR}/SampleCode/Hard_Fault_Sample)
add_subdirectory(${BSP_DIR}/SampleCode/Template)
add_subdirectory(${BSP_DIR}/SampleCode/ISP)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver)







