cmake_minimum_required(VERSION 3.5)

project(serial_port LANGUAGES CXX)

# Check static library or executable
if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set(SERIALPORT_AS_SUBDIRECTORY TRUE)
endif()

# Source files
set(SERIALPORT_SOURCES_FILES)
LIST(APPEND SERIALPORT_SOURCES_FILES "./src/serial_port.cpp")
LIST(APPEND SERIALPORT_SOURCES_FILES "./src/serial_port_looper.cpp")

# Add source files and link
if (${SERIALPORT_AS_SUBDIRECTORY})
    # Static library
    add_library(${PROJECT_NAME} STATIC ${SERIALPORT_SOURCES_FILES})

    target_include_directories(${PROJECT_NAME}
        PUBLIC 
            ${PROJECT_SOURCE_DIR}/include/serial_port
    )
else()
    # executable
    add_executable(${PROJECT_NAME} ${SERIALPORT_SOURCES_FILES})
endif()