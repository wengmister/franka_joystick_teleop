// franka_joystick_control_client.cpp
// Copyright (c) 2025 - Refactored to use frankx
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
#include <frankx/frankx.hpp>
#include <Eigen/Dense>

// ANSI color codes for logging
#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */


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

class FrankxJoystickController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    JoystickCommand current_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    // Movement limits for smooth teleoperation
    const double MAX_LINEAR_VEL = 0.1;   // Conservative for smooth operation
    const double MAX_ANGULAR_VEL = 0.1;  // Conservative for smooth operation
    
    // Connection monitoring
    std::chrono::steady_clock::time_point last_command_time_;
    std::atomic<bool> connection_active_{false};
    const double CONNECTION_TIMEOUT_SEC = 0.5; // 500ms timeout
    
    // Motion control parameters for smooth teleoperation
    const double MOTION_UPDATE_RATE = 0.1;  // 100ms motions (larger, less frequent)
    const double CONTROL_RATE_MS = 100;     // Update every 100ms for smoother motion
    const double DYNAMIC_REL = 0.3;         // 30% of max dynamics for smooth motion
    
    // Convert joystick commands to Cartesian velocity
    frankx::Affine calculateTargetVelocity(const JoystickCommand& cmd) {
        // Apply deadband to prevent small noise (much smaller deadband)
        const double deadband = 0.001;
        auto apply_deadband = [deadband](double value) {
            return (std::abs(value) < deadband) ? 0.0 : value;
        };
        
        // Create velocity vector (translation + rotation)
        double linear_x = apply_deadband(cmd.linear_y) * MAX_LINEAR_VEL;   // Forward/backward
        double linear_y = apply_deadband(-cmd.linear_x) * MAX_LINEAR_VEL;  // Left/right
        double linear_z = apply_deadband(cmd.linear_z) * MAX_LINEAR_VEL;   // Up/down
        double angular_x = apply_deadband(cmd.angular_x) * MAX_ANGULAR_VEL;
        double angular_y = apply_deadband(cmd.angular_y) * MAX_ANGULAR_VEL;
        double angular_z = apply_deadband(cmd.angular_z) * MAX_ANGULAR_VEL;
        
        std::cout << YELLOW << "[DEBUG] Before Affine: linear=[" << linear_x << "," << linear_y << "," << linear_z 
                  << "] angular=[" << angular_x << "," << angular_y << "," << angular_z << "]" << RESET << std::endl;
        
        return frankx::Affine(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z);
    }
    
public:
    FrankxJoystickController() {
        setupNetworking();
        
        // Initialize connection tracking
        last_command_time_ = std::chrono::steady_clock::now();
        connection_active_ = false;
    }
    
    ~FrankxJoystickController() {
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
                    
                    // Update connection tracking
                    last_command_time_ = std::chrono::steady_clock::now();
                    bool has_motion = (std::abs(cmd.linear_x) > 0.01 || std::abs(cmd.linear_y) > 0.01 || 
                                     std::abs(cmd.linear_z) > 0.01 || std::abs(cmd.angular_x) > 0.01 || 
                                     std::abs(cmd.angular_y) > 0.01 || std::abs(cmd.angular_z) > 0.01);
                    
                    if (has_motion) {
                        connection_active_ = true;
                    }
                    
                    if (cmd.emergency_stop) {
                        emergency_stop_ = true;
                        std::cout << "Emergency stop received!" << std::endl;
                    }
                    
                    // Debug output for non-zero commands
                    if (has_motion) {
                        std::cout << CYAN << "[JOYSTICK] Input: [" << cmd.linear_x << "," << cmd.linear_y 
                                  << "," << cmd.linear_z << "," << cmd.angular_x << "," << cmd.angular_y << "," << cmd.angular_z << "]" << RESET << std::endl;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void run(const std::string& robot_ip) {
        try {
            std::cout << "Connecting to robot at " << robot_ip << std::endl;
            frankx::Robot robot(robot_ip);
            robot.setDefaultBehavior();
            robot.setDynamicRel(0.3); // Moderate dynamics for smooth teleoperation
            
            // Move to ready position
            auto ready_motion = movex::JointMotion(std::array<double, 7>{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}});
            std::cout << "WARNING: This example will move the robot!" << std::endl
                      << "Please make sure to have the user stop button at hand!" << std::endl
                      << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.move(ready_motion);
            std::cout << "Robot moved to ready position." << std::endl;
            
            std::cout << "Starting network thread..." << std::endl;
            std::thread network_thread(&FrankxJoystickController::networkThread, this);
            
            std::cout << "Starting joystick control. Move joystick to control robot." << std::endl;
            std::cout << "Press joystick B button for emergency stop." << std::endl;
            
            // Main teleoperation loop using frankx impedance control
            while (!emergency_stop_) {
                try {
                    // Get current joystick command  
                    JoystickCommand cmd;
                    {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        cmd = current_command_;
                    }
                    
                    // Check for emergency stop
                    if (cmd.emergency_stop) {
                        std::cout << "Emergency stop received!" << std::endl;
                        break;
                    }
                    
                    // Check connection timeout
                    auto now = std::chrono::steady_clock::now();
                    auto time_since_last_command = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_).count() / 1000.0;
                    
                    if (time_since_last_command > CONNECTION_TIMEOUT_SEC && connection_active_) {
                        std::cout << "Joystick connection lost - stopping motion" << std::endl;
                        connection_active_ = false;
                        continue;
                    }
                    
                    // Calculate target velocity from joystick input
                    frankx::Affine target_velocity = calculateTargetVelocity(cmd);
                    
                    // Check if there's significant motion command
                    bool has_motion = (target_velocity.translation().norm() > 1e-6 || 
                                     target_velocity.angles().norm() > 1e-6);
                    
                    // Reduce logging frequency - only log when motion state changes
                    static bool last_had_motion = false;
                    if (has_motion != last_had_motion) {
                        auto trans = target_velocity.translation();
                        auto angles = target_velocity.angles();
                        std::cout << BLUE << "[VELOCITY] Translation: [" << trans.x() << "," << trans.y() << "," << trans.z() 
                                  << "] Rotation: [" << angles.x() << "," << angles.y() << "," << angles.z() << "]" << RESET << std::endl;
                        std::cout << GREEN << "[MOTION] Has motion: " << (has_motion ? "YES" : "NO") 
                                  << " (trans_norm=" << target_velocity.translation().norm() 
                                  << ", rot_norm=" << target_velocity.angles().norm() << ")" << RESET << std::endl;
                        last_had_motion = has_motion;
                    }
                    
                    if (has_motion) {
                        // Use larger linear relative motion for smoother teleoperation
                        // Scale the velocity by update rate to get position increment
                        auto translation = target_velocity.translation() * MOTION_UPDATE_RATE;
                        auto rotation_angles = target_velocity.angles() * MOTION_UPDATE_RATE;
                        
                        // Create relative motion
                        frankx::Affine relative_motion(translation.x(), translation.y(), translation.z(),
                                                     rotation_angles.x(), rotation_angles.y(), rotation_angles.z());
                        
                        // Use LinearRelativeMotion for smooth incremental moves
                        auto linear_motion = movex::LinearRelativeMotion(relative_motion, 0.0, DYNAMIC_REL);
                        
                        // Execute the motion
                        static int motion_count = 0;
                        motion_count++;
                        if (motion_count % 5 == 0) { // Log every 5th motion
                            std::cout << GREEN << "[CONTROL] Executing motion #" << motion_count << " - increment: [" 
                                      << translation.x() << "," << translation.y() << "," << translation.z() << "]" << RESET << std::endl;
                        }
                        
                        bool motion_success = robot.move(linear_motion);
                        
                        if (!motion_success && motion_count % 5 == 0) {
                            std::cout << RED << "[CONTROL] Motion #" << motion_count << " failed" << RESET << std::endl;
                        }
                    } else {
                        // Only log when motion state changes (already handled above)
                    }
                    
                    // Control loop rate for smooth teleoperation
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)CONTROL_RATE_MS));
                    
                } catch (const franka::Exception& e) {
                    std::cout << "Motion exception: " << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    robot.recoverFromErrors();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
        FrankxJoystickController controller;
        controller.run(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}