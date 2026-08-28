cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(BSP_DIR ${CMAKE_CURRENT_LIST_DIR})

# ---------------------------------------------------------------------
if(C_COMPILER_ID STREQUAL ARMCLANG)
    set(CMAKE_C_COMPILER_FORCED 1)
    set(CMAKE_CXX_COMPILER_FORCED 1)

    if(DEFINED ENV{ARMCLANG_PATH} AND EXISTS "$ENV{ARMCLANG_PATH}")
        set(TOOLCHAIN_PATH "$ENV{ARMCLANG_PATH}")
    elseif(DEFINED ENV{LOCALAPPDATA} AND EXISTS "$ENV{LOCALAPPDATA}/Keil_v5/ARM/ARMCLANG")
        set(TOOLCHAIN_PATH "$ENV{LOCALAPPDATA}/Keil_v5/ARM/ARMCLANG")
    elseif(EXISTS "C:/Keil_v5/ARM/ARMCLANG")
        set(TOOLCHAIN_PATH "C:/Keil_v5/ARM/ARMCLANG")
    else()
        find_program(ARM_CC_SEARCH armclang)
        if(ARM_CC_SEARCH)
            get_filename_component(ARM_CC_DIR ${ARM_CC_SEARCH} DIRECTORY)
            get_filename_component(TOOLCHAIN_PATH ${ARM_CC_DIR} DIRECTORY)
        else()
            set(TOOLCHAIN_PATH "C:/Keil_v5/ARM/ARMCLANG")
        endif()
    endif()

    set(TOOLCHAIN_INCLUDE ${TOOLCHAIN_PATH}/include)
    find_program(ARM_CC NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED)
    find_program(ARM_CXX NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED)
    find_program(ARM_ASM NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED)
    find_program(ARM_AR NAMES armar PATHS ${TOOLCHAIN_PATH}/bin REQUIRED)
    find_program(ARM_LINK NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED)
    find_program(ARM_FROMELF NAMES fromelf PATHS ${TOOLCHAIN_PATH}/bin REQUIRED)

    set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
    set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")
    set(CMAKE_ASM_COMPILER ${ARM_ASM} CACHE STRING "")
    set(CMAKE_LINKER ${ARM_LINK} CACHE STRING "")

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

    set(CMAKE_C_FLAGS "-mcpu=cortex-m23 -xc -std=c99 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -g -fno-function-sections -I${BSP_DIR}/Library/CMSIS/Include")
    set(CMAKE_ASM_FLAGS "-mcpu=cortex-m23 --target=arm-arm-none-eabi -masm=auto -Wa,armasm,--diag_suppress=A1950W -gdwarf-4 -D__MICROLIB -I${BSP_DIR}/Library/CMSIS/Include")
    set(CMAKE_EXECUTABLE_SUFFIX ".axf")
endif()

# ---------------------------------------------------------------------
if(C_COMPILER_ID STREQUAL GCC)
    # Prevent CMake from testing the toolchain
    set(CMAKE_C_COMPILER_FORCED 1)
    set(CMAKE_CXX_COMPILER_FORCED 1)

    if(DEFINED ENV{GNU_ARM_PATH} AND EXISTS "$ENV{GNU_ARM_PATH}")
        set(TOOLCHAIN_PATH "$ENV{GNU_ARM_PATH}")
    else()
        find_program(ARM_CC_SEARCH arm-none-eabi-gcc)
        if(ARM_CC_SEARCH)
            get_filename_component(TOOLCHAIN_PATH ${ARM_CC_SEARCH} DIRECTORY)
        else()
            file(GLOB VCPKG_GCC_PATHS 
                "$ENV{USERPROFILE}/.vcpkg/artifacts/*/compilers.arm.arm.none.eabi.gcc/*/bin"
                "$ENV{USERPROFILE}/.vcpkg/artifacts/vcpkg-ce-default/compilers.arm.none.eabi.gcc/*/bin"
                "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/bin"
                "C:/Program Files/GNU Arm Embedded Toolchain/*/bin"
            )
            if(VCPKG_GCC_PATHS)
                list(GET VCPKG_GCC_PATHS 0 TOOLCHAIN_PATH)
            else()
                set(TOOLCHAIN_PATH "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin")
            endif()
        endif()
    endif()

    find_program(ARM_CC NAMES arm-none-eabi-gcc PATHS ${TOOLCHAIN_PATH} REQUIRED)
    find_program(ARM_CXX NAMES arm-none-eabi-g++ arm-none-eabi-gcc PATHS ${TOOLCHAIN_PATH} REQUIRED)
    find_program(ARM_OBJCOPY NAMES arm-none-eabi-objcopy PATHS ${TOOLCHAIN_PATH} REQUIRED)

    get_filename_component(TOOLCHAIN_BIN_DIR ${ARM_CC} DIRECTORY)
    get_filename_component(TOOLCHAIN_ROOT_DIR ${TOOLCHAIN_BIN_DIR} DIRECTORY)
    set(TOOLCHAIN_INCLUDE "${TOOLCHAIN_ROOT_DIR}/arm-none-eabi/include")

    set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
    set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_ROOT_DIR})

    set(C_FLAGS "-mcpu=cortex-m23 -mfloat-abi=soft -Wall -Wno-main -Wno-format -Wno-return-type -Wno-unused-but-set-variable -fdata-sections -ffunction-sections -fno-builtin -fshort-enums -funsigned-char -mthumb -DMBEDTLS_CONFIG_FILE=mbedtls_config.h")
    set(CMAKE_C_FLAGS "-std=gnu11 ${C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${C_FLAGS}")
    set(CMAKE_ASM_FLAGS "-mcpu=Cortex-M23 -g -mthumb -mfloat-abi=soft")
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
