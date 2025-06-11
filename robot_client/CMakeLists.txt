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

# Add default_motionas a library
add_library(default_motion src/default_motion.cpp)
target_link_libraries(default_motion Franka::Franka)
target_include_directories(default_motion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})

# Add executable
add_executable(franka_joystick_control_client src/franka_joystick_control_client.cpp)

# Link libraries
target_link_libraries(franka_joystick_control_client
    Franka::Franka
    Threads::Threads
    default_motion
    ${EIGEN3_LIBRARIES}
)

# Include directories
target_include_directories(franka_joystick_control_client PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Optional: Set runtime path for shared libraries
set_target_properties(franka_joystick_control_client PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
)