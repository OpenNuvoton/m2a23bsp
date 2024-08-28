cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)

# ---------------------------------------------------------------------
if(C_COMPILER_ID STREQUAL ARMCLANG)
    set(CMAKE_C_COMPILER_FORCED 1)
    set(CMAKE_CXX_COMPILER_FORCED 1)
    set(TOOLCHAIN_PATH C:/Keil_v5/ARM/ARMCLANG)
    set(TOOLCHAIN_INCLUDE C:/Keil_v5/ARM/ARMCLANG/include)
    find_program(ARM_CC NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
    find_program(ARM_CXX NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
    find_program(ARM_ASM NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
    find_program(ARM_AR NAMES armar PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
    find_program(ARM_LINK NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
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

    set(CMAKE_C_FLAGS "-mcpu=cortex-m23 -xc -std=c99 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -g -fno-function-sections -IC:/Keil_v5/ARM/CMSIS/Include")
    set(CMAKE_ASM_FLAGS "-mcpu=cortex-m23 --target=arm-arm-none-eabi -masm=auto -Wa,armasm,--diag_suppress=A1950W -gdwarf-4 -D__MICROLIB -IC:/Keil_v5/ARM/CMSIS/Include")
    set(CMAKE_EXECUTABLE_SUFFIX " .axf ")
endif()

# ---------------------------------------------------------------------
if(C_COMPILER_ID STREQUAL GCC)
    # Prevent CMake from testing the toolchain
    set(CMAKE_C_COMPILER_FORCED 1)
    set(CMAKE_CXX_COMPILER_FORCED 1)

    set(TOOLCHAIN_PATH "c:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin")
    set(TOOLCHAIN_INCLUDE "c:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/arm-none-eabi/include")

    # set(TOOLCHAIN_PATH " ${ENV_USER}/.vcpkg/artifacts/fd3d78d7/compilers.arm.arm.none.eabi.gcc/12.3.1/bin ")
    find_program(ARM_CC NAMES arm-none-eabi-gcc PATHS ${TOOLCHAIN_PATH} REQUIRED NO_DEFAULT_PATH)
    find_program(ARM_CXX NAMES arm-none-eabi-gcc PATHS ${TOOLCHAIN_PATH} REQUIRED NO_DEFAULT_PATH)
    find_program(ARM_OBJCOPY NAMES arm-none-eabi-objcopy PATHS ${TOOLCHAIN_PATH} REQUIRED NO_DEFAULT_PATH)

    set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING " ")
    set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING " ")

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

    set(C_FLAGS " -mcpu=cortex-m23 -mfloat-abi=soft -Wall -Wno-main -Wno-format -Wno-return-type -Wno-unused-but-set-variable -fdata-sections -ffunction-sections -fno-builtin -fshort-enums -funsigned-char -mthumb -DMBEDTLS_CONFIG_FILE=mbedtls_config.h ")
    set(CMAKE_C_FLAGS " -std=gnu11 ${C_FLAGS} ")
    set(CMAKE_CXX_FLAGS ${C_FLAGS})
    set(CMAKE_ASM_FLAGS " -mcpu=Cortex-M23 -g -mthumb -mfloat-abi=soft ")
endif()

project(m2a23bsp)

function(create_project PRJ)
    add_executable(${PRJ} ${ARGN})
    target_link_libraries(${PRJ} system_startup standard_driver)

    if(C_COMPILER_ID STREQUAL ARMCLANG)
        target_link_options(${PRJ}
            PUBLIC
            -mcpu=cortex-m23
            --target=arm-arm-none-eabi
            -Xlinker
        )

        add_custom_command(TARGET ${PRJ} POST_BUILD
            COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
            VERBATIM
        )
    endif()

    if(C_COMPILER_ID STREQUAL GCC)
        target_link_options(${PRJ}
            PUBLIC
            -T ${BSP_DIR}/Library/Device/Nuvoton/m2a23/Source/GCC/gcc_arm.ld
            -Xlinker
            --gc-sections
            -Wl,-Map=${PRJ}.map
            -gdwarf-3
            --specs=nano.specs
        )

        add_custom_command(TARGET ${PRJ} POST_BUILD
            COMMAND ${ARM_OBJCOPY} -O binary ${PRJ} ${PRJ}.bin
            VERBATIM
        )
    endif()
endfunction()

set(BSP_DIR ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_INCLUDE})
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/m2a23/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)
include_directories(${BSP_DIR}/Library/NuMaker/common)

# Build library
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/m2a23/Source)
add_subdirectory(${BSP_DIR}/Library/NuMaker/xmodem)

# Build sample code
# create_project(Hard_Fault_Sample ${BSP_DIR}/SampleCode/Hard_Fault_Sample/main.c)
# create_project(Template ${BSP_DIR}/SampleCode/Template/main.c)
add_subdirectory(${BSP_DIR}/SampleCode/Template)
add_subdirectory(${BSP_DIR}/SampleCode/Hard_Fault_Sample)
add_subdirectory(${BSP_DIR}/SampleCode/ISP)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver)
