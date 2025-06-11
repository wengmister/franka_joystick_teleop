// franka_joystick_control_client.cpp
// Copyright (c) 2025 - Based on Franka examples
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include "default_motion.h"
#include <ruckig/ruckig.hpp>


struct JoystickCommand {
    double linear_x = 0.0;
    double linear_y = 0.0; 
    double linear_z = 0.0;
    double angular_x = 0.0;
    double angular_y = 0.0;
    double angular_z = 0.0;
    bool emergency_stop = false;
    bool reset_pose = false;
};

class ModernFrankaController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    JoystickCommand current_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    // Movement limits
    const double MAX_LINEAR_VEL = 0.05;  // 5cm/s max as you set
    const double MAX_ANGULAR_VEL = 0.05;  // Keep angular at 0.05 rad/s max
    const double SMOOTHING_FACTOR = 0.05; // Much more aggressive smoothing (was 0.1)
    
    // Smoothed velocities for continuity
    std::array<double, 6> smoothed_velocities_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> prev_velocities_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    
public:
    ModernFrankaController() {
        setupNetworking();
    }
    
    ~ModernFrankaController() {
        running_ = false;
        close(server_socket_);
    }
    
    void setupNetworking() {
        server_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (server_socket_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }
        
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(PORT);
        
        if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            throw std::runtime_error("Failed to bind socket");
        }
        
        int flags = fcntl(server_socket_, F_GETFL, 0);
        fcntl(server_socket_, F_SETFL, flags | O_NONBLOCK);
        
        std::cout << "UDP server listening on port " << PORT << std::endl;
    }
    
    void networkThread() {
        char buffer[1024];
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        while (running_) {
            ssize_t bytes_received = recvfrom(server_socket_, buffer, sizeof(buffer), 0,
                                            (struct sockaddr*)&client_addr, &client_len);
            
            if (bytes_received > 0) {
                JoystickCommand cmd;
                if (sscanf(buffer, "%lf %lf %lf %lf %lf %lf %d %d",
                          &cmd.linear_x, &cmd.linear_y, &cmd.linear_z,
                          &cmd.angular_x, &cmd.angular_y, &cmd.angular_z,
                          (int*)&cmd.emergency_stop, (int*)&cmd.reset_pose) == 8) {
                    
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    current_command_ = cmd;
                    
                    if (cmd.emergency_stop) {
                        emergency_stop_ = true;
                        std::cout << "Emergency stop received!" << std::endl;
                    }
                    
                    // Debug output for non-zero commands
                    if (std::abs(cmd.linear_x) > 0.01 || std::abs(cmd.linear_y) > 0.01 || std::abs(cmd.linear_z) > 0.01) {
                        std::cout << "Joystick: Left[" << cmd.linear_x << "," << cmd.linear_y 
                                  << "] Right[" << cmd.linear_z << "] -> Robot[fwd=" << cmd.linear_y 
                                  << ", right=" << -cmd.linear_x << ", up=" << cmd.linear_z << "]" << std::endl;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void run(const std::string& robot_ip) {
        try {
            std::cout << "Connecting to robot at " << robot_ip << std::endl;
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);
            
            // First move to initial joint configuration
            std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            MotionGenerator motion_generator(0.5, q_goal);
            std::cout << "WARNING: This example will move the robot!" << std::endl
                      << "Please make sure to have the user stop button at hand!" << std::endl
                      << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;
            
            // Set collision behavior - more permissive for joystick control
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
            
            std::cout << "Starting network thread..." << std::endl;
            std::thread network_thread(&ModernFrankaController::networkThread, this);
            
            std::cout << "Starting joystick control. Move joystick to control robot." << std::endl;
            std::cout << "Press joystick B button for emergency stop." << std::endl;
            
            // Create velocity control callback
            auto callback_control = [this](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::CartesianVelocities {
                
                // Check for emergency stop
                if (emergency_stop_) {
                    std::cout << "Emergency stop activated!" << std::endl;
                    franka::CartesianVelocities stop_velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                    return franka::MotionFinished(stop_velocities);
                }
                
                // Get current joystick command
                JoystickCommand cmd;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    cmd = current_command_;
                }
                
                // Convert joystick commands to target velocities with corrected mapping
                // Left stick: up/down = forward/backward (Y axis), left/right = left/right (X axis)
                double target_v_x = cmd.linear_y * MAX_LINEAR_VEL;  // Up/down -> forward/backward
                double target_v_y = -cmd.linear_x * MAX_LINEAR_VEL; // Left/right -> left/right (negated)
                double target_v_z = cmd.linear_z * MAX_LINEAR_VEL;  // Right stick vertical -> up/down
                double target_omega_x = cmd.angular_x * MAX_ANGULAR_VEL;
                double target_omega_y = cmd.angular_y * MAX_ANGULAR_VEL;
                double target_omega_z = cmd.angular_z * MAX_ANGULAR_VEL;
                
                // Apply exponential smoothing to prevent discontinuities
                // More aggressive smoothing for stability
                smoothed_velocities_[0] += SMOOTHING_FACTOR * (target_v_x - smoothed_velocities_[0]);
                smoothed_velocities_[1] += SMOOTHING_FACTOR * (target_v_y - smoothed_velocities_[1]);
                smoothed_velocities_[2] += SMOOTHING_FACTOR * (target_v_z - smoothed_velocities_[2]);
                smoothed_velocities_[3] += SMOOTHING_FACTOR * (target_omega_x - smoothed_velocities_[3]);
                smoothed_velocities_[4] += SMOOTHING_FACTOR * (target_omega_y - smoothed_velocities_[4]);
                smoothed_velocities_[5] += SMOOTHING_FACTOR * (target_omega_z - smoothed_velocities_[5]);
                
                // Apply deadzone to smoothed values
                for (int i = 0; i < 6; i++) {
                    if (std::abs(smoothed_velocities_[i]) < 0.002) { // 2mm/s deadzone
                        smoothed_velocities_[i] = 0.0;
                    }
                }
                
                // Additional acceleration limiting - clamp velocity changes
                const double MAX_VEL_CHANGE = 0.001; // Max change per cycle: 1mm/s
                
                for (int i = 0; i < 6; i++) {
                    double vel_change = smoothed_velocities_[i] - prev_velocities_[i];
                    if (vel_change > MAX_VEL_CHANGE) {
                        smoothed_velocities_[i] = prev_velocities_[i] + MAX_VEL_CHANGE;
                    } else if (vel_change < -MAX_VEL_CHANGE) {
                        smoothed_velocities_[i] = prev_velocities_[i] - MAX_VEL_CHANGE;
                    }
                    prev_velocities_[i] = smoothed_velocities_[i];
                }
                
                franka::CartesianVelocities output = {{
                    smoothed_velocities_[0], smoothed_velocities_[1], smoothed_velocities_[2],
                    smoothed_velocities_[3], smoothed_velocities_[4], smoothed_velocities_[5]
                }};
                
                return output;
            };
            
            std::cout << "Joystick control active! Use joystick to move robot." << std::endl;
            
            // Control loop with automatic restart after errors
            while (!emergency_stop_) {
                try {
                    // Start external velocity control loop
                    bool motion_finished = false;
                    auto active_control = robot.startCartesianVelocityControl(
                        research_interface::robot::Move::ControllerMode::kJointImpedance);
                    
                    while (!motion_finished && !emergency_stop_) {
                        auto read_once_return = active_control->readOnce();
                        auto robot_state = read_once_return.first;
                        auto duration = read_once_return.second;
                        auto cartesian_velocities = callback_control(robot_state, duration);
                        motion_finished = cartesian_velocities.motion_finished;
                        active_control->writeOnce(cartesian_velocities);
                    }
                    
                    if (motion_finished) {
                        std::cout << "Motion finished normally." << std::endl;
                        break;
                    }
                    
                } catch (const franka::ControlException& e) {
                    std::cout << "Control exception: " << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    robot.automaticErrorRecovery();
                    
                    // Reset smoothed velocities but keep prev_velocities to avoid discontinuity
                    std::fill(smoothed_velocities_.begin(), smoothed_velocities_.end(), 0.0);
                    // DON'T reset prev_velocities_ - let them naturally decay to zero
                    
                    std::cout << "Restarting control loop..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Longer pause
                }
            }
            
            std::cout << "Joystick control finished." << std::endl;
            running_ = false;
            network_thread.join();
            
        } catch (const franka::Exception& e) {
            std::cout << "Franka exception: " << e.what() << std::endl;
            running_ = false;
        } catch (const std::exception& e) {
            std::cout << "Standard exception: " << e.what() << std::endl;
            running_ = false;
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    
    try {
        ModernFrankaController controller;
        controller.run(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}