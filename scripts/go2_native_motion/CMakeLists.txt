cmake_minimum_required(VERSION 3.16)
project(go2_native_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(go2_native_motion_core STATIC native_motion_core.cpp)
target_include_directories(go2_native_motion_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(go2_native_motion_core PRIVATE -Wall -Wextra)

add_executable(go2_native_motion_selftest selftest.cpp)
target_link_libraries(go2_native_motion_selftest PRIVATE go2_native_motion_core)
target_compile_options(go2_native_motion_selftest PRIVATE -Wall -Wextra)

find_package(unitree_sdk2 CONFIG QUIET)
if(unitree_sdk2_FOUND)
  add_executable(go2_native_motion main.cpp)
  target_link_libraries(go2_native_motion
    PRIVATE go2_native_motion_core unitree_sdk2)
  target_compile_options(go2_native_motion PRIVATE -Wall -Wextra)

  add_executable(go2_native_readonly_query readonly_query.cpp)
  target_link_libraries(go2_native_readonly_query PRIVATE unitree_sdk2)
  target_compile_options(go2_native_readonly_query PRIVATE -Wall -Wextra)
else()
  message(WARNING
    "unitree_sdk2 not found; building only go2_native_motion_selftest")
endif()
