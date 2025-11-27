cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME R1_chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        chassis_control/Src/chassis_path.c
        chassis_control/Src/path.c
        chassis_control/Src/chassis_pid.c
        Driver_Layer/Src/locator_driver.c
        chassis_control/Src/chassis_driver.c
        Driver_remote/Src/remote_driver.c
        Driver_Motor/Src/dji_3508_2006_motor.c
        Driver_Motor/Src/vesc.c
        Task/Src/Task_LED.c
        Task/Src/Task_Printf.c
        BSP/Src/bsp_dwt.c
        BSP/Src/bsp_can.c
        Math_User/Src/pid.c
        Task/Src/Task_command.c
        Task/Src/Task_chassis.c
        Task/Inc/Task_chassis.h


)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/chassis_control/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Driver_Layer/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Driver_remote/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Driver_Motor/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Math_User/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Task/Inc

        # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# STM32G474xx (Cortex-M4F) 应该启用 HARD FPU
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16")
# 使能printf浮点数支持
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

