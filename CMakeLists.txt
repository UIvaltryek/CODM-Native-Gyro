cmake_minimum_required(VERSION 3.22)
project(native_gyro)

# Download the Android-fixed fork of Dobby
include(FetchContent)
FetchContent_Declare(
  dobby
  GIT_REPOSITORY https://github.com/chiteroman/Dobby.git
)
FetchContent_MakeAvailable(dobby)

# Compile our module
add_library(native_gyro SHARED main.cpp)
target_include_directories(native_gyro PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# The chiteroman fork uses 'dobby_static' instead of 'dobby'
target_link_libraries(native_gyro android log dobby_static)
