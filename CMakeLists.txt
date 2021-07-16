cmake_minimum_required(VERSION 3.5)

project(serial_port LANGUAGES CXX)

# Check static library or executable
if(NOT ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set(SERIALPORT_AS_SUBDIRECTORY TRUE)
endif()

# Add source files and link
if (${SERIALPORT_AS_SUBDIRECTORY})
    # ---Static library---

    #******Source******
    add_subdirectory("./src")

else()
    # ---Build example---

    #******Source******
    add_subdirectory("./src")

    #******Example******
    add_subdirectory("./examples")
    
endif()