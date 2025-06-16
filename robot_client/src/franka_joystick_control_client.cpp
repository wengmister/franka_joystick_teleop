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
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <Eigen/Dense>
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
    
    // Movement limits for impedance control
    const double MAX_LINEAR_VEL = 0.15;  // 15cm/s max 
    const double MAX_ANGULAR_VEL = 0.05;  // 0.05 rad/s max
    
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
                {{40.0, 40.0, 30.0, 30.0, 30.0, 30.0, 25.0}}, {{40.0, 40.0, 30.0, 30.0, 30.0, 30.0, 25.0}},
                {{40.0, 40.0, 30.0, 30.0, 30.0, 30.0, 25.0}}, {{40.0, 40.0, 30.0, 30.0, 30.0, 30.0, 25.0}},
                {{40.0, 40.0, 40.0, 30.0, 30.0, 25.0}}, {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},
                {{40.0, 40.0, 40.0, 30.0, 30.0, 25.0}}, {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}});
            
            // Configure custom end-effector (0.7kg mass)
            // TODO: Update COM and inertia matrix when available
            std::array<double, 16> EE_T_K = {{1.0, 0.0, 0.0, 0.0,  // Identity transformation for now
                                              0.0, 1.0, 0.0, 0.0,
                                              0.0, 0.0, 1.0, 0.0,
                                              0.0, 0.0, 0.0, 1.0}};
            robot.setEE(EE_T_K);
            
            // Set end-effector mass and inertia
            double ee_mass = 0.7;  // 0.7kg custom end-effector
            std::array<double, 3> ee_com = {{0.0, 0.0, 0.05}};  // COM offset from flange (update when known)
            std::array<double, 9> ee_inertia = {{0.01, 0.0, 0.0,   // Inertia matrix (update when calculated)
                                                 0.0, 0.01, 0.0,
                                                 0.0, 0.0, 0.01}};
            robot.setLoad(ee_mass, ee_com, ee_inertia);
            
            std::cout << "Configured custom end-effector: mass=" << ee_mass << "kg" << std::endl;
            
            std::cout << "Starting network thread..." << std::endl;
            std::thread network_thread(&ModernFrankaController::networkThread, this);
            
            std::cout << "Starting joystick control. Move joystick to control robot." << std::endl;
            std::cout << "Press joystick B button for emergency stop." << std::endl;
            
            // Impedance control parameters
            Eigen::Matrix<double, 6, 6> stiffness = Eigen::MatrixXd::Zero(6, 6);
            Eigen::Matrix<double, 6, 6> damping = Eigen::MatrixXd::Zero(6, 6);
            
            // Increased stiffness for more responsive control
            stiffness.topLeftCorner(3, 3) << 1500.0 * Eigen::MatrixXd::Identity(3, 3);  // Translational: 1200 N/m
            stiffness.bottomRightCorner(3, 3) << 100.0 * Eigen::MatrixXd::Identity(3, 3); // Rotational: 100 Nm/rad
            
            // Reduced damping for more natural motion
            damping.topLeftCorner(3, 3) << 1.5 * sqrt(1500.0) * Eigen::MatrixXd::Identity(3, 3);
            damping.bottomRightCorner(3, 3) << 1.5 * sqrt(100.0) * Eigen::MatrixXd::Identity(3, 3);
            
            // Initial target pose
            franka::Model model = robot.loadModel();
            franka::RobotState initial_state = robot.readOnce();
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Vector3d target_position = initial_transform.translation();
            Eigen::Quaterniond target_orientation(initial_transform.linear());
            
            // Smoothing variables for target updates
            Eigen::Vector3d smoothed_velocity = Eigen::Vector3d::Zero();
            Eigen::Vector3d smoothed_angular_velocity = Eigen::Vector3d::Zero();
            const double VELOCITY_SMOOTHING = 0.1;  // Smoothing factor for target velocity

            // Create impedance control callback
            auto callback_control = [this, &model, &stiffness, &damping, &target_position, &target_orientation, 
                                   &smoothed_velocity, &smoothed_angular_velocity, VELOCITY_SMOOTHING]
                                  (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                
                // Check for emergency stop
                if (emergency_stop_) {
                    std::cout << "Emergency stop activated!" << std::endl;
                    return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
                }
                
                // Get current joystick command
                JoystickCommand cmd;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    cmd = current_command_;
                }
                
                // Smooth joystick input to target velocities
                double dt = period.toSec();
                Eigen::Vector3d target_vel;
                target_vel[0] = cmd.linear_y * MAX_LINEAR_VEL;    // forward/backward
                target_vel[1] = -cmd.linear_x * MAX_LINEAR_VEL;   // left/right  
                target_vel[2] = cmd.linear_z * MAX_LINEAR_VEL;    // up/down
                
                Eigen::Vector3d target_angular_vel;
                target_angular_vel[0] = cmd.angular_x * MAX_ANGULAR_VEL;
                target_angular_vel[1] = cmd.angular_y * MAX_ANGULAR_VEL;
                target_angular_vel[2] = cmd.angular_z * MAX_ANGULAR_VEL;
                
                // Apply exponential smoothing to velocities
                smoothed_velocity = (1.0 - VELOCITY_SMOOTHING) * smoothed_velocity + VELOCITY_SMOOTHING * target_vel;
                smoothed_angular_velocity = (1.0 - VELOCITY_SMOOTHING) * smoothed_angular_velocity + VELOCITY_SMOOTHING * target_angular_vel;
                
                // Apply deadzone to smoothed velocities
                for (int i = 0; i < 3; i++) {
                    if (std::abs(smoothed_velocity[i]) < 0.002) {
                        smoothed_velocity[i] = 0.0;
                    }
                    if (std::abs(smoothed_angular_velocity[i]) < 0.005) {
                        smoothed_angular_velocity[i] = 0.0;
                    }
                }
                
                // Update target position using smoothed velocities
                target_position += smoothed_velocity * dt;
                
                // Update target orientation using smoothed angular velocities
                if (smoothed_angular_velocity.norm() > 1e-6) {
                    double angle = smoothed_angular_velocity.norm() * dt;
                    Eigen::Vector3d axis = smoothed_angular_velocity.normalized();
                    Eigen::AngleAxisd delta_rot(angle, axis);
                    target_orientation = target_orientation * delta_rot;
                    target_orientation.normalize();
                }
                
                // Get current robot dynamics
                std::array<double, 7> coriolis_array = model.coriolis(robot_state);
                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                
                // Convert to Eigen
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
                
                // Current pose
                Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                Eigen::Vector3d position(transform.translation());
                Eigen::Quaterniond orientation(transform.linear());
                
                // Compute pose error
                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << position - target_position;
                
                // Orientation error (quaternion)
                if (target_orientation.coeffs().dot(orientation.coeffs()) < 0.0) {
                    orientation.coeffs() << -orientation.coeffs();
                }
                Eigen::Quaterniond error_quaternion(orientation.inverse() * target_orientation);
                error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                error.tail(3) << -transform.linear() * error.tail(3);
                
                // Compute impedance control law
                Eigen::VectorXd wrench_cartesian = -stiffness * error - damping * (jacobian * dq);
                
                // Convert to joint torques
                Eigen::VectorXd tau_task = jacobian.transpose() * wrench_cartesian;
                Eigen::VectorXd tau_d = tau_task + coriolis;
                
                // Convert to array
                std::array<double, 7> tau_d_array;
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
                
                return franka::Torques(tau_d_array);
            };
            
            std::cout << "Joystick control active! Use joystick to move robot." << std::endl;
            
            // Control loop with automatic restart after errors
            while (!emergency_stop_) {
                try {
                    // Use torque control for impedance control
                    robot.control(callback_control);
                    break; // Exit loop when control finishes
                    
                } catch (const franka::ControlException& e) {
                    std::cout << "Control exception: " << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    robot.automaticErrorRecovery();
                    
                    // Reset target to current pose to avoid jumps
                    franka::RobotState current_state = robot.readOnce();
                    Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(current_state.O_T_EE.data()));
                    target_position = current_transform.translation();
                    target_orientation = Eigen::Quaterniond(current_transform.linear());
                    
                    // Reset smoothed velocities
                    smoothed_velocity.setZero();
                    smoothed_angular_velocity.setZero();
                    
                    std::cout << "Restarting control loop..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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