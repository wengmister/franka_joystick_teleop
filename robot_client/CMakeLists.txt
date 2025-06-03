cmake_minimum_required(VERSION 3.10)
project(franka_joystick_control_client_client)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set CMAKE_PREFIX_PATH to find your libfranka installation
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};$ENV{HOME}/ws/franka/install")

# Find required packages
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)

# Add executable
add_executable(franka_joystick_control_client src/franka_joystick_control_client.cpp)

# Link libraries
target_link_libraries(franka_joystick_control_client
    Franka::Franka
    Threads::Threads
)

# Optional: Set runtime path for shared libraries
set_target_properties(franka_joystick_control_client PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)