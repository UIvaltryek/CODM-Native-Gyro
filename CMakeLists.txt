cmake_minimum_required(VERSION 3.22)
project(native_gyro)

# Automatically download and configure the Dobby hooking library
include(FetchContent)
FetchContent_Declare(
  dobby
  GIT_REPOSITORY https://github.com/jmpews/Dobby.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(dobby)

# Compile our module
add_library(native_gyro SHARED main.cpp)
target_include_directories(native_gyro PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(native_gyro android log dobby)
