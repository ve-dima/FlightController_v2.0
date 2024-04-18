cmake_minimum_required(VERSION 3.16)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CXX_CF 
        -Og
        -g -gdwarf-2

        -fdata-sections
        -ffunction-sections
        -fno-common
        -fno-exceptions
        -ffreestanding
        -flto
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        -fmessage-length=0
        -fsingle-precision-constant

        -Wall
        -Werror
)

project(FlightController C CXX ASM)

set(MCU -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

file(GLOB_RECURSE SOURCES
        "CMSIS_G4/Source/system_stm32g4xx.c"
        "CMSIS_G4/Source/Templates/gcc/startup_stm32g431xx.s"
        "src/*"
        "lib/*"
        "platform/stm32/*.cpp"
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/platform/stm32/STM32G431CBTx_FLASH.ld)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

target_compile_options(${PROJECT_NAME}.elf
        PUBLIC
        --specs=nano.specs
        # --specs=nosys.specs
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
 
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
 
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT
        "Building ${HEX_FILE} 
        Building ${BIN_FILE}"
)

# # For pc
# file(GLOB_RECURSE SOURCES
#         "src/*"
#         "platform/pc/*.cpp"
# )
# set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/platform/pc/sections.ld)

# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT}") 

# add_executable(${PROJECT_NAME} ${SOURCES} ${LINKER_SCRIPT})

# target_compile_options(${PROJECT_NAME}
#         PUBLIC
#         ${CXX_CF}
# )

# target_include_directories(${PROJECT_NAME}
#         PUBLIC
#                 "include"
#                 "lib"
#                 "src/*"
#                 "platform/pc"
# )
# target_link_options(${PROJECT_NAME} PUBLIC -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)