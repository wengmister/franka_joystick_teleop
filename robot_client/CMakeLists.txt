cmake_minimum_required(VERSION 3.10)
project(franka_joystick_control)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set CMAKE_PREFIX_PATH to find your libfranka installation
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};$ENV{HOME}/ws/franka/install")

# Find required packages
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# Force static libraries globally
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libraries" FORCE)

# Disable tests and Python bindings for all subdirectories
set(BUILD_TESTS OFF CACHE BOOL "Disable all tests" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable all testing" FORCE)
set(ENABLE_TESTS OFF CACHE BOOL "Disable all tests" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "Disable Python bindings" FORCE)
set(WITH_PYTHON OFF CACHE BOOL "Disable Python support" FORCE)

# Add frankx as a subdirectory (this is the cleanest approach)
add_subdirectory(frankx)

# Add default_motion as a STATIC library
add_library(default_motion STATIC src/default_motion.cpp)
target_link_libraries(default_motion Franka::Franka)
target_include_directories(default_motion PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${EIGEN3_INCLUDE_DIR}
)

# Add kinematics library (if you have it) as STATIC
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics.cpp")
    add_library(kinematics STATIC src/kinematics.cpp)
    target_link_libraries(kinematics Franka::Franka Eigen3::Eigen)
    target_include_directories(kinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

# Add executable
add_executable(franka_joystick_control_client src/franka_joystick_control_client.cpp)

# Link libraries - frankx includes all necessary dependencies
target_link_libraries(franka_joystick_control_client
    frankx          # This includes Franka::Franka and other dependencies
    Threads::Threads
    Eigen3::Eigen
    -static-libgcc
    -static-libstdc++
)

# Include directories - frankx provides its own headers
target_include_directories(franka_joystick_control_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/frankx/include
)

# Set RPATH for the executable
set_target_properties(franka_joystick_control_client PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/frankx/build"
)