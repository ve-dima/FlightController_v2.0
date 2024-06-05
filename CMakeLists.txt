cmake_minimum_required(VERSION 3.16)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CXX_CF 
        -Og
        # -O2
        -g -gdwarf-2
        -Wall
        -Werror
)
set(MCU -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)


project (mavlink)
add_library(mavlink INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lib/mavlink)
target_compile_options(mavlink INTERFACE -Wno-address-of-packed-member)
target_include_directories(mavlink INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lib/mavlink)
target_compile_definitions(mavlink INTERFACE 
        MAVLINK_NO_SIGN_PACKET
        MAVLINK_NO_SIGNATURE_CHECK
        MAVLINK_USE_CONVENIENCE_FUNCTIONS
)

project(lwrb)
add_subdirectory(lib/lwrb/lwrb)
target_compile_definitions(lwrb PUBLIC LWRB_DISABLE_ATOMIC)
target_compile_options(lwrb PRIVATE
        -O2
        ${MCU}
)

project(FlightController C CXX ASM)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/platform/stm32/STM32G431CBTx_FLASH.ld)
file(GLOB_RECURSE SOURCES
        "CMSIS_G4/Source/system_stm32g4xx.c"
        "CMSIS_G4/Source/Templates/gcc/startup_stm32g431xx.s"
        "src/*"
        "lib/Algo/*"
        "lib/ArduinoAPI/*"
        "lib/SRT/*.cpp"
        "platform/stm32/*.cpp"
)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
target_compile_options(${PROJECT_NAME}.elf
        PUBLIC
        --specs=nano.specs
        --specs=nosys.specs
        -fdata-sections
        -ffunction-sections
        -fno-common
        -fno-exceptions
        -ffreestanding
        -flto
        -fmessage-length=0
        -fsingle-precision-constant
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        ${MCU}
        ${CXX_CF}
)
target_include_directories(${PROJECT_NAME}.elf
        PUBLIC
        ${CMAKE_SOURCE_DIR}/CMSIS/CMSIS/Core/Include
        ${CMAKE_SOURCE_DIR}/CMSIS_G4/Include
)
target_compile_definitions(${PROJECT_NAME}.elf
        PUBLIC
        STM32G431xx
        F_CPU=160000000
        HSE_VALUE=8000000
        MAVLINK_COMM_NUM_BUFFERS=2
)
 
target_link_options(${PROJECT_NAME}.elf PUBLIC -specs=nano.specs)
target_link_options(${PROJECT_NAME}.elf PUBLIC -specs=nosys.specs)
target_link_options(${PROJECT_NAME}.elf PUBLIC ${MCU})
target_link_options(${PROJECT_NAME}.elf PUBLIC -T ${LINKER_SCRIPT})
target_link_options(${PROJECT_NAME}.elf PUBLIC -Wl,--gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)

target_include_directories(${PROJECT_NAME}.elf 
        PUBLIC
                "include"
                "src"
                "lib"
                "platform/stm32"
)
target_link_libraries (${PROJECT_NAME}.elf Eigen3::Eigen)
target_link_libraries (${PROJECT_NAME}.elf lwrb)
target_link_libraries (${PROJECT_NAME}.elf mavlink)
target_precompile_headers(mavlink
        INTERFACE "include/mavlink_bridge_header.hpp")
 
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
 
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT
        "Building ${HEX_FILE} 
        Building ${BIN_FILE}"
)