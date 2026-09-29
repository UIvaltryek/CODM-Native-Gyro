cmake_minimum_required(VERSION 3.22)
project(native_gyro)

# BUGFIX: Force Clang to run the C preprocessor on assembly files.
# This stops it from accidentally trying to compile Apple/iOS syntax on Android.
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -x assembler-with-cpp -D__linux__=1 -D__ANDROID__=1")

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
