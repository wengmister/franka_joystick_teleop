// Modern franka_joystick_control_client.cpp - Using external control loop
// Copyright (c) 2024 - Based on Franka examples
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
#include "examples_common.h"

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
    const double MAX_LINEAR_VEL = 0.05;  // 5cm/s max
    const double MAX_ANGULAR_VEL = 0.1;  // 0.1 rad/s max
    
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
                        std::cout << "Joystick: [" << cmd.linear_x << "," << cmd.linear_y << "," << cmd.linear_z << "]" << std::endl;
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
                
                // Convert joystick commands to velocities
                double v_x = cmd.linear_x * MAX_LINEAR_VEL;
                double v_y = cmd.linear_y * MAX_LINEAR_VEL;
                double v_z = cmd.linear_z * MAX_LINEAR_VEL;
                double omega_x = cmd.angular_x * MAX_ANGULAR_VEL;
                double omega_y = cmd.angular_y * MAX_ANGULAR_VEL;
                double omega_z = cmd.angular_z * MAX_ANGULAR_VEL;
                
                franka::CartesianVelocities output = {{v_x, v_y, v_z, omega_x, omega_y, omega_z}};
                return output;
            };
            
            // Start external velocity control loop
            bool motion_finished = false;
            auto active_control = robot.startCartesianVelocityControl(
                research_interface::robot::Move::ControllerMode::kJointImpedance);
                
            std::cout << "Joystick control active! Use joystick to move robot." << std::endl;
            
            while (!motion_finished && !emergency_stop_) {
                try {
                    auto read_once_return = active_control->readOnce();
                    auto robot_state = read_once_return.first;
                    auto duration = read_once_return.second;
                    auto cartesian_velocities = callback_control(robot_state, duration);
                    motion_finished = cartesian_velocities.motion_finished;
                    active_control->writeOnce(cartesian_velocities);
                    
                } catch (const franka::ControlException& e) {
                    std::cout << "Control exception: " << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    robot.automaticErrorRecovery();
                    break;
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