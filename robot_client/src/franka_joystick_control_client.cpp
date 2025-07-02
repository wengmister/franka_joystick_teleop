// Fixed franka_joystick_control_client.cpp - Real-time optimized smooth control
// Copyright (c) 2024 - Based on Franka examples
//
// JOYSTICK CONTROL MAPPING:
// =============================
// D-pad (axes 6,7):    Robot translation (forward/back, left/right)
// Left stick (axes 0,1):   End-effector orientation (roll/pitch)  
// Right stick (axes 3,4):  Z-movement (vertical) + yaw rotation
//
// Optimized for real-time performance with simplified smoothing
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
    double axis0 = 0.0;  // left horizontal -> roll
    double axis1 = 0.0;  // left vertical -> pitch  
    double axis2 = 0.0;  // left trigger (unused)
    double axis3 = 0.0;  // right horizontal -> yaw
    double axis4 = 0.0;  // right vertical -> z movement
    double axis5 = 0.0;  // right trigger (unused)
    double axis6 = 0.0;  // dpad horizontal -> right/left
    double axis7 = 0.0;  // dpad vertical -> forward/backward
    
    bool emergency_stop = false;
    bool reset_pose = false;
};

// Tuned parameters for responsive yet smooth control
struct SimpleTeleopParams {
    double max_linear_velocity = 0.08;     // Increased to 8cm/s max for better responsiveness
    double max_angular_velocity = 0.04;    // Increased to 0.04 rad/s max
    double velocity_smoothing = 0.5;       // Reduced from 0.7 for more responsiveness
    double position_smoothing = 0.6;       // Reduced from 0.8 for more responsiveness
    double deadzone_linear = 0.003;        // Reduced deadzone for better sensitivity
    double deadzone_angular = 0.008;       // Reduced deadzone for better sensitivity
    
    // Tuned acceleration limiting for responsiveness
    double max_linear_acceleration = 0.02;   // Increased from 0.01 to 2cm/s²
    double max_angular_acceleration = 0.01;  // Increased from 0.005 to 0.01 rad/s²
    
    // Workspace limits (keeping safe)
    Eigen::Vector3d workspace_min{-0.2, -0.2, -0.1};
    Eigen::Vector3d workspace_max{0.4, 0.2, 0.4};
};

class SimpleSmoothController {
private:
    SimpleTeleopParams config_;
    
    // Minimal state for real-time performance
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    Eigen::Affine3d virtual_target_;
    Eigen::Affine3d initial_pose_;
    
    // Pre-computed for performance
    double vel_alpha_;
    double pos_alpha_;
    
public:
    SimpleSmoothController() : current_velocity_(Eigen::Vector3d::Zero()),
                              current_angular_velocity_(Eigen::Vector3d::Zero()) {
        vel_alpha_ = 1.0 - config_.velocity_smoothing;
        pos_alpha_ = 1.0 - config_.position_smoothing;
    }
    
    void setInitialPose(const Eigen::Affine3d& pose) {
        initial_pose_ = pose;
        virtual_target_ = pose;
        current_velocity_.setZero();
        current_angular_velocity_.setZero();
    }
    
    // Simplified update optimized for real-time
    Eigen::Affine3d updateAndGetTarget(const JoystickCommand& cmd, double dt) {
        if (dt <= 0.0 || dt > 0.01) return virtual_target_; // Safety check
        
        // Calculate target velocities (minimal computation)
        Eigen::Vector3d target_vel;
        target_vel[0] = cmd.axis7 * config_.max_linear_velocity;    // forward/backward
        target_vel[1] = -cmd.axis6 * config_.max_linear_velocity;   // left/right
        target_vel[2] = cmd.axis4 * config_.max_linear_velocity;    // up/down
        
        Eigen::Vector3d target_angular_vel;
        target_angular_vel[0] = cmd.axis0 * config_.max_angular_velocity;  // roll
        target_angular_vel[1] = cmd.axis1 * config_.max_angular_velocity;  // pitch
        target_angular_vel[2] = cmd.axis3 * config_.max_angular_velocity;  // yaw
        
        // Reduced debug output for better performance during tuning
        static int debug_counter = 0;
        debug_counter++;
        bool show_debug = (debug_counter % 1000 == 0) || 
                         (target_vel.norm() > 0.005 && debug_counter % 200 == 0) || 
                         (target_angular_vel.norm() > 0.005 && debug_counter % 200 == 0);
        
        if (show_debug) {
            std::cout << "TARGET VEL: Lin=[" << target_vel.transpose() << "] norm=" << target_vel.norm() 
                      << " Ang=[" << target_angular_vel.transpose() << "] norm=" << target_angular_vel.norm() << std::endl;
        }
        
        // Simple deadzone (fast) - reduced debug spam
        for (int i = 0; i < 3; i++) {
            if (std::abs(target_vel[i]) < config_.deadzone_linear) {
                target_vel[i] = 0.0;
            }
            if (std::abs(target_angular_vel[i]) < config_.deadzone_angular) {
                target_angular_vel[i] = 0.0;
            }
        }
        
        // Show post-deadzone values only when significant
        if (show_debug && (target_vel.norm() > 0.001 || target_angular_vel.norm() > 0.001)) {
            std::cout << "AFTER DEADZONE: Lin=[" << target_vel.transpose() << "] norm=" << target_vel.norm() 
                      << " Ang=[" << target_angular_vel.transpose() << "] norm=" << target_angular_vel.norm() << std::endl;
        }
        
        // Simple acceleration limiting
        Eigen::Vector3d accel = (target_vel - current_velocity_) / dt;
        if (accel.norm() > config_.max_linear_acceleration) {
            accel = accel.normalized() * config_.max_linear_acceleration;
            target_vel = current_velocity_ + accel * dt;
        }
        
        Eigen::Vector3d angular_accel = (target_angular_vel - current_angular_velocity_) / dt;
        if (angular_accel.norm() > config_.max_angular_acceleration) {
            angular_accel = angular_accel.normalized() * config_.max_angular_acceleration;
            target_angular_vel = current_angular_velocity_ + angular_accel * dt;
        }
        
        // Smooth velocities (fast exponential smoothing)
        current_velocity_ = vel_alpha_ * target_vel + config_.velocity_smoothing * current_velocity_;
        current_angular_velocity_ = vel_alpha_ * target_angular_vel + config_.velocity_smoothing * current_angular_velocity_;
        
        // Debug current velocities and position changes (reduced frequency)
        if (show_debug && (current_velocity_.norm() > 0.001 || current_angular_velocity_.norm() > 0.001)) {
            std::cout << "CURRENT VEL: Lin=[" << current_velocity_.transpose() << "] norm=" << current_velocity_.norm() 
                      << " Ang=[" << current_angular_velocity_.transpose() << "] norm=" << current_angular_velocity_.norm() << std::endl;
        }
        
        // Store previous position for debugging
        Eigen::Vector3d prev_position = virtual_target_.translation();
        
        // Integrate velocities to position
        virtual_target_.translation() += current_velocity_ * dt;
        
        // Debug position change (only for significant changes)
        Eigen::Vector3d pos_change = virtual_target_.translation() - prev_position;
        if (pos_change.norm() > 0.0005) {  // Only show changes > 0.5mm
            std::cout << "POSITION CHANGE: [" << pos_change.transpose() << "] norm=" << pos_change.norm() << "mm" << std::endl;
        }
        
        // Integrate angular velocity (simplified)
        if (current_angular_velocity_.norm() > 1e-6) {
            double angle = current_angular_velocity_.norm() * dt;
            Eigen::Vector3d axis = current_angular_velocity_.normalized();
            Eigen::AngleAxisd delta_rot(angle, axis);
            virtual_target_.linear() = virtual_target_.linear() * delta_rot.toRotationMatrix();
        }
        
        // Enforce workspace limits (fast)
        Eigen::Vector3d relative_pos = virtual_target_.translation() - initial_pose_.translation();
        Eigen::Vector3d clamped_pos = relative_pos;
        for (int i = 0; i < 3; i++) {
            clamped_pos[i] = std::max(config_.workspace_min[i], 
                                     std::min(config_.workspace_max[i], relative_pos[i]));
        }
        
        // Debug workspace clamping
        if ((clamped_pos - relative_pos).norm() > 1e-6) {
            std::cout << "WORKSPACE CLAMP: " << relative_pos.transpose() << " -> " << clamped_pos.transpose() << std::endl;
        }
        
        virtual_target_.translation() = initial_pose_.translation() + clamped_pos;
        
        return virtual_target_;
    }
    
    void resetToInitialPose() {
        virtual_target_ = initial_pose_;
        current_velocity_.setZero();
        current_angular_velocity_.setZero();
    }
    
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getCurrentVelocities() const {
        return {current_velocity_, current_angular_velocity_};
    }
};

class TeleoperationController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    JoystickCommand current_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    // Simple smooth controller
    SimpleSmoothController smooth_controller_;
    
    // Pre-computed target for real-time thread
    std::atomic<bool> target_updated_{false};
    Eigen::Affine3d computed_target_;
    std::mutex target_mutex_;
    
public:
    TeleoperationController() {
        setupNetworking();
    }
    
    ~TeleoperationController() {
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
        
        std::cout << "Network thread started, waiting for joystick data..." << std::endl;
        
        while (running_) {
            ssize_t bytes_received = recvfrom(server_socket_, buffer, sizeof(buffer), 0,
                                            (struct sockaddr*)&client_addr, &client_len);
            
            if (bytes_received > 0) {
                buffer[bytes_received] = '\0';
                
                JoystickCommand cmd;
                int parsed_count = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %d %d",
                          &cmd.axis0, &cmd.axis1, &cmd.axis2, &cmd.axis3, &cmd.axis4, &cmd.axis5, &cmd.axis6, &cmd.axis7,
                          (int*)&cmd.emergency_stop, (int*)&cmd.reset_pose);
                
                if (parsed_count == 10) {
                    // Debug: Show received commands occasionally
                    static int debug_count = 0;
                    debug_count++;
                    if (debug_count % 100 == 0 || cmd.reset_pose || cmd.emergency_stop) {
                        std::cout << "UDP RX: [" << cmd.axis0 << "," << cmd.axis1 << "," << cmd.axis2 << "," 
                                  << cmd.axis3 << "," << cmd.axis4 << "," << cmd.axis5 << "," 
                                  << cmd.axis6 << "," << cmd.axis7 << "] E:" << cmd.emergency_stop 
                                  << " R:" << cmd.reset_pose << std::endl;
                    }
                    
                    // Show non-zero commands immediately
                    if (std::abs(cmd.axis0) > 0.01 || std::abs(cmd.axis1) > 0.01 || std::abs(cmd.axis3) > 0.01 ||
                        std::abs(cmd.axis4) > 0.01 || std::abs(cmd.axis6) > 0.01 || std::abs(cmd.axis7) > 0.01) {
                        std::cout << "JOYSTICK INPUT: Left[" << cmd.axis0 << "," << cmd.axis1 << "] "
                                  << "Right[" << cmd.axis3 << "," << cmd.axis4 << "] "
                                  << "Dpad[" << cmd.axis6 << "," << cmd.axis7 << "]" << std::endl;
                    }
                    
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    current_command_ = cmd;
                    
                    if (cmd.emergency_stop) {
                        emergency_stop_ = true;
                        std::cout << "Emergency stop received!" << std::endl;
                    }
                    
                    if (cmd.reset_pose) {
                        std::cout << "Reset pose requested!" << std::endl;
                    }
                } else {
                    static int error_count = 0;
                    error_count++;
                    if (error_count % 100 == 0) {
                        std::cout << "UDP WARNING: Malformed packets: " << error_count << std::endl;
                        std::cout << "  Raw buffer: [" << buffer << "]" << std::endl;
                        std::cout << "  Parsed count: " << parsed_count << std::endl;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    franka::CartesianPose eigenToCartesianPose(const Eigen::Affine3d& pose) {
        std::array<double, 16> pose_array;
        Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = pose.matrix();
        return franka::CartesianPose(pose_array);
    }
    
    void run(const std::string& robot_ip) {
        try {
            std::cout << "Connecting to robot at " << robot_ip << std::endl;
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);
            
            // Move to initial joint configuration
            std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            MotionGenerator motion_generator(0.5, q_goal);
            std::cout << "WARNING: This example will move the robot!" << std::endl
                      << "Please make sure to have the user stop button at hand!" << std::endl
                      << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;
            
            // Configure robot for smooth control
            robot.setCollisionBehavior(
                {{50.0, 50.0, 40.0, 40.0, 40.0, 40.0, 30.0}}, {{50.0, 50.0, 40.0, 40.0, 40.0, 40.0, 30.0}},
                {{50.0, 50.0, 40.0, 40.0, 40.0, 40.0, 30.0}}, {{50.0, 50.0, 40.0, 40.0, 40.0, 40.0, 30.0}},
                {{50.0, 50.0, 50.0, 40.0, 40.0, 30.0}}, {{40.0, 40.0, 40.0, 30.0, 30.0, 30.0}},
                {{50.0, 50.0, 50.0, 40.0, 40.0, 30.0}}, {{40.0, 40.0, 40.0, 30.0, 30.0, 30.0}});
            
            // Lower impedance for smoother motion
            robot.setCartesianImpedance({{600, 600, 600, 60, 60, 60}});
            
            // Configure custom end-effector
            std::array<double, 16> EE_T_K = {{1.0, 0.0, 0.0, 0.0,
                                              0.0, 1.0, 0.0, 0.0,
                                              0.0, 0.0, 1.0, 0.0,
                                              0.0, 0.0, 0.0, 1.0}};
            robot.setEE(EE_T_K);
            
            double ee_mass = 0.7;
            std::array<double, 3> ee_com = {{0.0, 0.0, 0.05}};
            std::array<double, 9> ee_inertia = {{0.01, 0.0, 0.0,
                                                 0.0, 0.01, 0.0,
                                                 0.0, 0.0, 0.01}};
            robot.setLoad(ee_mass, ee_com, ee_inertia);
            
            std::cout << "Robot configured for smooth teleoperation." << std::endl;
            
            // Start networking
            std::thread network_thread(&TeleoperationController::networkThread, this);
            
            // Run optimized teleoperation
            this->runOptimizedTeleop(robot);
            
            std::cout << "Teleoperation finished." << std::endl;
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
    
private:
    void runOptimizedTeleop(franka::Robot& robot) {
        std::cout << "Starting optimized real-time teleoperation..." << std::endl;
        
        // Error recovery and settling
        try {
            robot.automaticErrorRecovery();
        } catch (const franka::Exception& e) {
            std::cout << "Error recovery: " << e.what() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Initialize controller
        franka::RobotState reference_state = robot.readOnce();
        Eigen::Affine3d current_pose(Eigen::Matrix4d::Map(reference_state.O_T_EE_d.data()));
        smooth_controller_.setInitialPose(current_pose);
        
        std::cout << "Controller initialized. Starting motion..." << std::endl;
        
        static int iteration_count = 0;
        std::atomic<bool> reset_requested{false};
        
        // Optimized motion generator - minimal computation
        auto optimized_generator = [this, &iteration_count, &reset_requested]
                                  (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
            
            iteration_count++;
            double dt = period.toSec();
            
            // Emergency stop check (fastest possible)
            if (emergency_stop_) {
                return franka::MotionFinished(eigenToCartesianPose(smooth_controller_.updateAndGetTarget(JoystickCommand{}, dt)));
            }
            
            // Start control after short stability period
            if (iteration_count > 5 && dt > 0.0 && dt < 0.01) {
                // Get joystick command with minimal locking
                JoystickCommand cmd;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    cmd = current_command_;
                }
                
                // Handle reset
                if (cmd.reset_pose && !reset_requested) {
                    smooth_controller_.resetToInitialPose();
                    reset_requested = true;
                }
                if (!cmd.reset_pose) {
                    reset_requested = false;
                }
                
                // Update and get target (optimized single call)
                Eigen::Affine3d target = smooth_controller_.updateAndGetTarget(cmd, dt);
                
                // Debug occasionally and when there's significant input
                if (iteration_count % 1000 == 0) {  // Every 10 seconds
                    auto [lin_vel, ang_vel] = smooth_controller_.getCurrentVelocities();
                    std::cout << "Iter " << iteration_count << " - Vel norms: Lin=" << lin_vel.norm() << " Ang=" << ang_vel.norm() << std::endl;
                    std::cout << "  Current cmd: [" << cmd.axis0 << "," << cmd.axis1 << "," << cmd.axis3 << "," << cmd.axis4 << "," << cmd.axis6 << "," << cmd.axis7 << "]" << std::endl;
                }
                
                return eigenToCartesianPose(target);
            }
            
            // During stability period, return current target
            return eigenToCartesianPose(smooth_controller_.updateAndGetTarget(JoystickCommand{}, dt));
        };
        
        try {
            std::cout << "Starting optimized motion generator..." << std::endl;
            robot.control(optimized_generator);
            std::cout << "Motion generator finished normally." << std::endl;
        } catch (const franka::ControlException& e) {
            std::cout << "Control exception: " << e.what() << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    
    try {
        TeleoperationController controller;
        controller.run(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}