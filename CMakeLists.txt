cmake_minimum_required(VERSION 3.16)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_SYSTEM_NAME Generic)
# find_program(CMAKE_CXX_CPPCHECK NAMES cppcheck)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CXX_CF 
        -Og
        -g -gdwarf-2
        -Wall
        -Werror
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

project (mavlink)
add_library(mavlink INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lib/mavlink)
target_include_directories(mavlink INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/lib/mavlink)
target_compile_options(mavlink INTERFACE -Wno-address-of-packed-member)
target_compile_definitions(mavlink INTERFACE MAVLINK_NO_SIGN_PACKET MAVLINK_NO_SIGNATURE_CHECK MAVLINK_USE_CONVENIENCE_FUNCTIONS)

project(FlightController C CXX ASM)
set(MCU -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/platform/stm32/STM32G431CBTx_FLASH.ld)
file(GLOB_RECURSE SOURCES
        "CMSIS_G4/Source/system_stm32g4xx.c"
        "CMSIS_G4/Source/Templates/gcc/startup_stm32g431xx.s"
        "src/*"
        "lib/*"
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
        -DSTM32G431xx
        -DF_CPU=16000000
)
 
target_link_options(${PROJECT_NAME}.elf PUBLIC -specs=nano.specs)
target_link_options(${PROJECT_NAME}.elf PUBLIC -specs=nosys.specs)
target_link_options(${PROJECT_NAME}.elf PUBLIC ${MCU})
target_link_options(${PROJECT_NAME}.elf PUBLIC -T ${LINKER_SCRIPT})
target_link_options(${PROJECT_NAME}.elf PUBLIC -Wl,--gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)

target_include_directories(${PROJECT_NAME}.elf 
        PUBLIC
                "include"
                "lib"
                "src/*"
                "platform/stm32"
)
target_link_libraries (${PROJECT_NAME}.elf Eigen3::Eigen)
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

# For pc
file(GLOB_RECURSE SOURCES
        "src/*"
        "lib/*"
        "platform/pc/*.cpp"
)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/platform/pc/sections.ld)

add_executable(${PROJECT_NAME} ${SOURCES} ${LINKER_SCRIPT})

set_target_properties(${PROJECT_NAME} PROPERTIES CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT}") 
target_compile_options(${PROJECT_NAME}
        PUBLIC
        ${CXX_CF}
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
                "include"
                "lib"
                "src/*"
                "platform/pc"
)
target_link_options(${PROJECT_NAME} PUBLIC -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)