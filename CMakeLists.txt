cmake_minimum_required(VERSION 3.24)

project(serial_port LANGUAGES CXX)

# Create static library
add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
        include/serial_port/serial_port_info.h
        include/serial_port/serial_port.h
        src/serial_port.cpp
        src/serial_port_info.cpp
)

# Make sure the compiler can find include files
target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
)

target_compile_features(${PROJECT_NAME}
    PUBLIC
        cxx_std_20
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        _CRT_SECURE_NO_WARNINGS
)