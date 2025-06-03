// Simple franka_joystick_control_client.cpp - Minimal version for debugging
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
#include <franka/exception.h>
#include <franka/robot.h>

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

class SimpleFrankaController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    JoystickCommand current_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    std::array<double, 16> initial_pose_;
    
public:
    SimpleFrankaController() {
        setupNetworking();
    }
    
    ~SimpleFrankaController() {
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
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void setDefaultBehavior(franka::Robot& robot) {
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    }
    
    void run(const std::string& robot_ip) {
        try {
            std::cout << "Connecting to robot..." << std::endl;
            franka::Robot robot(robot_ip);
            std::cout << "Robot connected successfully" << std::endl;
            
            setDefaultBehavior(robot);
            std::cout << "Default behavior set" << std::endl;
            
            std::cout << "Starting network thread..." << std::endl;
            std::thread network_thread(&SimpleFrankaController::networkThread, this);
            std::cout << "Network thread started" << std::endl;
            
            std::cout << "WARNING: Robot will move based on joystick input!" << std::endl;
            std::cout << "Press Enter to start control loop..." << std::endl;
            std::cin.ignore();
            
            std::cout << "Starting control loop..." << std::endl;
            
            double time = 0.0;
            
            robot.control([&](const franka::RobotState& robot_state,
                             franka::Duration period) -> franka::CartesianPose {
                
                try {
                    time += period.toSec();
                    
                    if (time == 0.0) {
                        initial_pose_ = robot_state.O_T_EE;
                        std::cout << "Initial pose captured" << std::endl;
                        return initial_pose_;
                    }
                    
                    // Check for emergency stop
                    if (emergency_stop_) {
                        std::cout << "Emergency stop activated!" << std::endl;
                        return franka::MotionFinished(robot_state.O_T_EE);
                    }
                    
                    // Get current joystick command
                    JoystickCommand cmd;
                    {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        cmd = current_command_;
                    }
                    
                    // For now, just stay at initial position until we get this working
                    std::array<double, 16> target = initial_pose_;
                    
                    // Add tiny movement if joystick is moved
                    double dt = period.toSec();
                    double movement_scale = 0.01; // Very small movements
                    
                    if (std::abs(cmd.linear_x) > 0.01) {
                        target[12] += cmd.linear_x * movement_scale * dt;
                        std::cout << "Moving X: " << cmd.linear_x * movement_scale * dt << std::endl;
                    }
                    if (std::abs(cmd.linear_y) > 0.01) {
                        target[13] += cmd.linear_y * movement_scale * dt;
                        std::cout << "Moving Y: " << cmd.linear_y * movement_scale * dt << std::endl;
                    }
                    if (std::abs(cmd.linear_z) > 0.01) {
                        target[14] += cmd.linear_z * movement_scale * dt;
                        std::cout << "Moving Z: " << cmd.linear_z * movement_scale * dt << std::endl;
                    }
                    
                    return target;
                    
                } catch (const std::exception& e) {
                    std::cout << "Exception in control loop: " << e.what() << std::endl;
                    return franka::MotionFinished(robot_state.O_T_EE);
                } catch (...) {
                    std::cout << "Unknown exception in control loop!" << std::endl;
                    return franka::MotionFinished(robot_state.O_T_EE);
                }
            });
            
            std::cout << "Control loop finished" << std::endl;
            network_thread.join();
            
        } catch (const franka::Exception& e) {
            std::cout << "Franka exception: " << e.what() << std::endl;
            running_ = false;
        } catch (const std::exception& e) {
            std::cout << "Standard exception: " << e.what() << std::endl;
            running_ = false;
        } catch (...) {
            std::cout << "Unknown exception caught in main!" << std::endl;
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
        std::cout << "Creating controller..." << std::endl;
        SimpleFrankaController controller;
        std::cout << "Controller created, starting..." << std::endl;
        controller.run(argv[1]);
        std::cout << "Controller finished" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error in main: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "Unknown error in main!" << std::endl;
        return -1;
    }
    
    return 0;
}