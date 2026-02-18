cmake_minimum_required(VERSION 3.10)
project("visualizator")

set(CMAKE_CXX_STANDART 11)

find_package(PCL REQUIRED)

find_library(SDK_LIB
    NAMES livox_lidar_sdk_static
    HINTS /usr/local/lib)

add_executable(visualizer
    main.cpp
    SafeQueue.h
    drawsphere.h)

target_link_libraries(visualizer
    ${PCL_LIBRARIES}
    ${SDK_LIB})

