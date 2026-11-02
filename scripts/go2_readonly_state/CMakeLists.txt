cmake_minimum_required(VERSION 3.16)
project(go2_readonly_state LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(unitree_sdk2 CONFIG REQUIRED)

add_executable(go2_readonly_state main.cpp)
target_link_libraries(go2_readonly_state PRIVATE unitree_sdk2)
