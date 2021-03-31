cmake_minimum_required(VERSION 3.5)

project(serial_port LANGUAGES CXX)

# Check static library or executable
if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set(SERIALPORT_AS_SUBDIRECTORY TRUE)
endif()

# Create source files list
set(SERIALPORT_SOURCES_FILES)
LIST(APPEND SERIALPORT_SOURCES_FILES "./src/serial_port_info.cpp")
LIST(APPEND SERIALPORT_SOURCES_FILES "./src/serial_port.cpp")
LIST(APPEND SERIALPORT_SOURCES_FILES "./src/serial_port_looper.cpp")

# Add source files and link
if (${SERIALPORT_AS_SUBDIRECTORY})
    # ---Static library---

    # Add sources files to library
    add_library(${PROJECT_NAME} STATIC ${SERIALPORT_SOURCES_FILES})

    # Link header files
    target_include_directories(${PROJECT_NAME}
        PUBLIC 
            ${PROJECT_SOURCE_DIR}/include/serial_port
    )
else()
    # ---Build example---

    # Add examples
    LIST(APPEND SERIALPORT_SOURCES_FILES "./examples/example.cpp")
    LIST(APPEND SERIALPORT_SOURCES_FILES "./examples/eg1_print_ports.cpp")
    LIST(APPEND SERIALPORT_SOURCES_FILES "./examples/eg2_read.cpp")

    # Add sources files
    add_executable(${PROJECT_NAME} ${SERIALPORT_SOURCES_FILES})

    # Link header files
    target_include_directories(${PROJECT_NAME}
        PRIVATE 
            ${PROJECT_SOURCE_DIR}/include/serial_port
            ${PROJECT_SOURCE_DIR}/examples
    )
endif()