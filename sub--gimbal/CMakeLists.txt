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
set(CMAKE_PROJECT_NAME C_Board_CubeMX_Example)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Lib/GCC
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Application/DetectTask/detect_task.c
    Application/AimAssistTask/aimassist_task.c
    Application/GimbalTask/gimbal_task.c
    Application/INSTask/ins_task.c
    Application/INSTask/QuaternionEKF.c
    Application/ShootTask/shoot_task.c

    Bsp/JudgementSystem/judgement_info.c
    Bsp/Motor/DJI/motor.c
    Bsp/RemoteControl/remote_control.c
    Bsp/MCU/bsp_CAN.c
    Bsp/MCU/bsp_PWM.c
    Bsp/MCU/bsp_dwt.c
    Bsp/MCU/bsp_usart_idle.c
    Bsp/BMI088/BMI088driver.c
    Bsp/BMI088/BMI088Middleware.c

    Middlewares/Filter/KalmanFilter/kalman_filter.c
    Middlewares/Filter/LowPassFilter/filter32.c
    Middlewares/Algorithm/Include/user_lib.c
    Middlewares/Algorithm/SystemIdentification/system_identification.c
    Middlewares/Algorithm/SystemIdentification/transfer_function.c
    Middlewares/Controller/controller.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Application/AimAssistTask
    Application/DetectTask
    Application/GimbalTask
    Application/INSTask
    Application/ShootTask
    Bsp/BMI088
    Bsp/JudgementSystem
    Bsp/MCU
    Bsp/Motor/DJI
    Bsp/RemoteControl
    Middlewares/Algorithm/Include
    Middlewares/Algorithm/SystemIdentification
    Middlewares/Controller
    Middlewares/Filter/KalmanFilter
    Middlewares/Filter/LowPassFilter
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    arm_cortexM4lf_math
    m
    nosys
)

# Produce ELF-derived flashable artifacts
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ELF size report, HEX and BIN images"
)
