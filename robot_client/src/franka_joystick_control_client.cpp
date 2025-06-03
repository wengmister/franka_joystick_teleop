#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <chrono>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

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

class FrankaJoystickController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    JoystickCommand current_command_;
    std::mutex command_mutex_;
    
    // Network
    int server_socket_;
    const int PORT = 8888;
    
    // Control parameters
    const double MAX_LINEAR_VEL = 0.1;  // m/s
    const double MAX_ANGULAR_VEL = 0.5; // rad/s
    const double CONTROL_FREQ = 1000.0; // Hz
    
    std::array<double, 16> initial_pose_;
    std::array<double, 16> target_pose_;
    
public:
    FrankaJoystickController() {
        setupNetworking();
    }
    
    ~FrankaJoystickController() {
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
        
        // Set socket to non-blocking
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
                // Parse the received command (simple format: "lx ly lz ax ay az estop reset")
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
    
    std::array<double, 16> matrixMultiply(const std::array<double, 16>& pose, 
                                         const std::array<double, 16>& delta) {
        std::array<double, 16> result = pose;
        
        // Simple approach: apply translational delta directly
        result[12] += delta[12]; // x
        result[13] += delta[13]; // y  
        result[14] += delta[14]; // z
        
        // For rotation, we'll keep it simple and just apply small rotational changes
        // In a production system, you'd want proper rotation matrix composition
        
        return result;
    }
    
    void run(const std::string& robot_ip) {
        try {
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);
            
            std::cout << "Connected to robot at " << robot_ip << std::endl;
            std::cout << "Starting network thread..." << std::endl;
            
            std::thread network_thread(&FrankaJoystickController::networkThread, this);
            
            std::cout << "WARNING: Robot will move based on joystick input!" << std::endl;
            std::cout << "Press Enter to start control loop..." << std::endl;
            std::cin.ignore();
            
            double time = 0.0;
            auto start_time = std::chrono::steady_clock::now();
            
            robot.control([&](const franka::RobotState& robot_state,
                             franka::Duration period) -> franka::CartesianPose {
                
                time += period.toSec();
                
                if (time == 0.0) {
                    initial_pose_ = robot_state.O_T_EE;
                    target_pose_ = initial_pose_;
                    std::cout << "Initial pose captured" << std::endl;
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
                
                // Reset pose if requested
                if (cmd.reset_pose) {
                    target_pose_ = initial_pose_;
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    current_command_.reset_pose = false;
                    std::cout << "Pose reset to initial position" << std::endl;
                }
                
                // Apply velocity commands to target pose
                double dt = period.toSec();
                std::array<double, 16> delta = {0};
                
                // Translational velocities
                delta[12] = cmd.linear_x * MAX_LINEAR_VEL * dt;  // x
                delta[13] = cmd.linear_y * MAX_LINEAR_VEL * dt;  // y
                delta[14] = cmd.linear_z * MAX_LINEAR_VEL * dt;  // z
                
                // Apply delta to target pose
                target_pose_ = matrixMultiply(target_pose_, delta);
                
                // Simple safety limits (keep robot within reasonable bounds)
                double max_translation = 0.5; // 50cm from initial position
                if (std::abs(target_pose_[12] - initial_pose_[12]) > max_translation) {
                    target_pose_[12] = initial_pose_[12] + 
                        std::copysign(max_translation, target_pose_[12] - initial_pose_[12]);
                }
                if (std::abs(target_pose_[13] - initial_pose_[13]) > max_translation) {
                    target_pose_[13] = initial_pose_[13] + 
                        std::copysign(max_translation, target_pose_[13] - initial_pose_[13]);
                }
                if (std::abs(target_pose_[14] - initial_pose_[14]) > max_translation) {
                    target_pose_[14] = initial_pose_[14] + 
                        std::copysign(max_translation, target_pose_[14] - initial_pose_[14]);
                }
                
                return target_pose_;
            });
            
            network_thread.join();
            
        } catch (const franka::Exception& e) {
            std::cout << "Franka exception: " << e.what() << std::endl;
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
        FrankaJoystickController controller;
        controller.run(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}