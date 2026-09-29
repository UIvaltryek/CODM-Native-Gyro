cmake_minimum_required(VERSION 3.22)
project(native_gyro)

# Include the local Dobby folder that our workflow will download
add_subdirectory(Dobby)

# Compile our module
add_library(native_gyro SHARED main.cpp)

# Point to Dobby's include folder so main.cpp can find dobby.h
target_include_directories(native_gyro PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/Dobby/include)

# Link Dobby to our module
target_link_libraries(native_gyro android log dobby)
