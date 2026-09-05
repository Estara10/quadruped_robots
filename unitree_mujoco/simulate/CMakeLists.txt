cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

# Enables `ctest` and defines BUILD_TESTING (default ON).
include(CTest)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
)

set(SIM_DEPENDENCIES
    pthread
    rt
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2
    boost_program_options
    fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_include_directories(unitree_mujoco PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

# P1-08 observability instrumentation + model probe.
# The registered sim-clock test is the v2 contract test (test/p1_08_sim_clock_test.cpp).
# The old v1 test (test/p1_08_sim_clock_v1_legacy.cpp) is NOT registered.
add_executable(p1_08_sim_clock_test test/p1_08_sim_clock_test.cpp)
target_link_libraries(p1_08_sim_clock_test PRIVATE rt Threads::Threads)
add_test(NAME p1_08_sim_clock_test COMMAND p1_08_sim_clock_test)

# C++ -> Python integration bridge (no MuJoCo): invoked by the Python test to
# create a REAL SimClockWriter on a unique temp shm that Python then reads.
add_executable(p1_08_sim_clock_bridge test/p1_08_sim_clock_bridge.cpp)
target_link_libraries(p1_08_sim_clock_bridge PRIVATE rt Threads::Threads)

add_executable(p1_08_model_probe test/p1_08_model_probe.cpp)
target_link_libraries(p1_08_model_probe PRIVATE mujoco::mujoco)

# P1-10 offline initial-state binding probe. This follows the real startup
# construction path but does not step the model or start a runtime capture.
add_executable(p1_10_initial_state_probe test/p1_10_initial_state_probe.cpp)
target_include_directories(p1_10_initial_state_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
target_link_libraries(p1_10_initial_state_probe PRIVATE mujoco::mujoco)

add_executable(p1_10_model_fingerprint_test test/p1_10_model_fingerprint_test.cpp)
target_include_directories(p1_10_model_fingerprint_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
target_link_libraries(p1_10_model_fingerprint_test PRIVATE mujoco::mujoco)

add_executable(p1_09o_joinable_thread_test test/p1_09o_joinable_thread.cpp)
add_executable(p1_09p_reload_barrier_test test/p1_09p_reload_barrier.cpp)
add_executable(p1_09u_bridge_lifecycle_state_test test/p1_09u_bridge_lifecycle_state.cpp)
add_executable(p1_09v_terminal_invariant_test test/p1_09v_terminal_invariant.cpp)
