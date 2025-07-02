// Modern franka_joystick_control_client.cpp - Using external control loop
// Copyright (c) 2024 - Based on Franka examples
//
// NEW JOYSTICK CONTROL MAPPING:
// =============================
// D-pad (axes 6,7):    Robot translation (forward/back, left/right)
// Left stick (axes 0,1):   End-effector orientation (roll/pitch)  
// Right stick (axes 3,4):  Z-movement (vertical) + yaw rotation
// Triggers (axes 2,5):     Unused
//
// Motion is smoothed and step-limited to prevent discontinuities
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
    // 8 joystick axes
    double axis0 = 0.0;  // left horizontal -> yaw
    double axis1 = 0.0;  // left vertical -> pitch  
    double axis2 = 0.0;  // left trigger (unused)
    double axis3 = 0.0;  // right horizontal -> roll
    double axis4 = 0.0;  // right vertical -> z movement
    double axis5 = 0.0;  // right trigger (unused)
    double axis6 = 0.0;  // dpad horizontal -> right/left
    double axis7 = 0.0;  // dpad vertical -> forward/backward
    
    bool emergency_stop = false;
    bool reset_pose = false;
};

struct TeleopParams {
    double max_linear_velocity = 0.10;   // Reduced from 0.15 to 0.10 (10cm/s max)
    double max_angular_velocity = 0.05;  // 0.05 rad/s max
    double velocity_smoothing = 0.25;    // Increased smoothing factor from 0.15 to 0.25
    double deadzone_linear = 0.005;      // Increased linear deadzone from 0.002 to 0.005
    double deadzone_angular = 0.01;      // Increased angular deadzone from 0.005 to 0.01
    
    // Additional smoothing parameters for joystick noise
    double noise_threshold = 0.02;       // Threshold for aggressive smoothing of tiny values
    double noise_smoothing = 0.8;        // Very aggressive smoothing for noisy inputs
    
    // Input filtering parameters (smooth raw joystick before velocity conversion)
    double input_smoothing = 0.8;        // Much more aggressive smoothing factor (was 0.4)
    double input_max_change = 0.01;     // Much smaller maximum allowed change (was 0.02)
    
    // Maximum step size constraints (prevents discontinuities)
    double max_step_size = 0.005;        // Reduced: Maximum position change per step (5mm)
    double max_angular_step = 0.005;     // Reduced: Maximum angular change per step (0.005 rad ≈ 0.28°)
    
    // Workspace limits (relative to initial pose)
    Eigen::Vector3d workspace_min{-0.3, -0.3, -0.2};  // 30cm workspace
    Eigen::Vector3d workspace_max{0.3, 0.3, 0.2};
};

class TeleoperationController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    JoystickCommand current_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    // Control parameters
    TeleopParams config_;
    
    // Robot state tracking
    Eigen::Affine3d initial_pose_;
    Eigen::Affine3d virtual_target_;
    Eigen::Vector3d smoothed_velocity_;
    Eigen::Vector3d smoothed_angular_velocity_;
    
    // Smoothed joystick inputs (filter raw inputs before velocity conversion)
    JoystickCommand smoothed_joystick_inputs_;
    
public:
    TeleoperationController() : smoothed_velocity_(Eigen::Vector3d::Zero()), 
                               smoothed_angular_velocity_(Eigen::Vector3d::Zero()) {
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
        
        while (running_) {
            ssize_t bytes_received = recvfrom(server_socket_, buffer, sizeof(buffer), 0,
                                            (struct sockaddr*)&client_addr, &client_len);
            
            if (bytes_received > 0) {
                // Null-terminate buffer to prevent reading garbage data
                buffer[bytes_received] = '\0';
                
                JoystickCommand cmd;
                int parsed_count = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %d %d",
                          &cmd.axis0, &cmd.axis1, &cmd.axis2, &cmd.axis3, &cmd.axis4, &cmd.axis5, &cmd.axis6, &cmd.axis7,
                          (int*)&cmd.emergency_stop, (int*)&cmd.reset_pose);
                
                if (parsed_count == 10) {
                    // Debug: Show parsed values
                    static int debug_count = 0;
                    debug_count++;
                    if (debug_count % 100 == 0 || cmd.reset_pose || cmd.emergency_stop) {  // Every 100th message or if flags set
                        std::cout << "UDP DEBUG: Raw buffer: [" << buffer << "]" << std::endl;
                        std::cout << "UDP DEBUG: Parsed - Axes:[" << cmd.axis0 << "," << cmd.axis1 << "," << cmd.axis2 << "," << cmd.axis3 << "," << cmd.axis4 << "," << cmd.axis5 << "," << cmd.axis6 << "," << cmd.axis7 << "]" << std::endl;
                        std::cout << "UDP DEBUG: E-Stop:" << cmd.emergency_stop << " Reset:" << cmd.reset_pose << std::endl;
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
                    
                    // Debug output for non-zero commands
                    if (std::abs(cmd.axis0) > 0.01 || std::abs(cmd.axis1) > 0.01 || std::abs(cmd.axis2) > 0.01 ||
                        std::abs(cmd.axis3) > 0.01 || std::abs(cmd.axis4) > 0.01 || std::abs(cmd.axis6) > 0.01 ||
                        std::abs(cmd.axis7) > 0.01) {
                        std::cout << "Joystick: Left[" << cmd.axis0 << "," << cmd.axis1 << "," << cmd.axis2 << "]"
                                  << " Right[" << cmd.axis3 << "," << cmd.axis4 << "," << cmd.axis5 << "]"
                                  << " Dpad[" << cmd.axis6 << "," << cmd.axis7 << "]" << std::endl;
                    }
                } else {
                    std::cout << "UDP WARNING: Received malformed packet, parsed " << parsed_count << " values instead of 10" << std::endl;
                    std::cout << "UDP WARNING: Raw buffer: [" << buffer << "]" << std::endl;
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void resetVirtualTarget() {
        virtual_target_ = initial_pose_;
        smoothed_velocity_.setZero();
        smoothed_angular_velocity_.setZero();
    }
    
    void enforceWorkspaceLimits(Eigen::Affine3d& target) {
        // Calculate position relative to initial pose
        Eigen::Vector3d relative_pos = target.translation() - initial_pose_.translation();
        
        // Clamp to workspace limits
        for (int i = 0; i < 3; i++) {
            relative_pos[i] = std::max(config_.workspace_min[i], 
                                     std::min(config_.workspace_max[i], relative_pos[i]));
        }
        
        // Update target position
        target.translation() = initial_pose_.translation() + relative_pos;
    }
    
    void updateVirtualTarget(double dt) {
        // Get current joystick command
        JoystickCommand raw_cmd;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            raw_cmd = current_command_;
        }
        
        // Apply input filtering to smooth raw joystick inputs (prevents large jumps)
        smoothed_joystick_inputs_.axis0 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis0 + 
                                            config_.input_smoothing * raw_cmd.axis0;
        smoothed_joystick_inputs_.axis1 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis1 + 
                                            config_.input_smoothing * raw_cmd.axis1;
        smoothed_joystick_inputs_.axis2 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis2 + 
                                            config_.input_smoothing * raw_cmd.axis2;
        smoothed_joystick_inputs_.axis3 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis3 + 
                                             config_.input_smoothing * raw_cmd.axis3;
        smoothed_joystick_inputs_.axis4 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis4 + 
                                             config_.input_smoothing * raw_cmd.axis4;
        smoothed_joystick_inputs_.axis5 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis5 + 
                                             config_.input_smoothing * raw_cmd.axis5;
        smoothed_joystick_inputs_.axis6 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis6 + 
                                             config_.input_smoothing * raw_cmd.axis6;
        smoothed_joystick_inputs_.axis7 = (1.0 - config_.input_smoothing) * smoothed_joystick_inputs_.axis7 + 
                                             config_.input_smoothing * raw_cmd.axis7;
        
        // Apply maximum change limiting to smoothed inputs (prevent sudden jumps)
        auto clamp_change = [this](double current, double previous) -> double {
            double change = current - previous;
            if (std::abs(change) > config_.input_max_change) {
                return previous + std::copysign(config_.input_max_change, change);
            }
            return current;
        };
        
        static JoystickCommand prev_smoothed = smoothed_joystick_inputs_;
        smoothed_joystick_inputs_.axis0 = clamp_change(smoothed_joystick_inputs_.axis0, prev_smoothed.axis0);
        smoothed_joystick_inputs_.axis1 = clamp_change(smoothed_joystick_inputs_.axis1, prev_smoothed.axis1);
        smoothed_joystick_inputs_.axis2 = clamp_change(smoothed_joystick_inputs_.axis2, prev_smoothed.axis2);
        smoothed_joystick_inputs_.axis3 = clamp_change(smoothed_joystick_inputs_.axis3, prev_smoothed.axis3);
        smoothed_joystick_inputs_.axis4 = clamp_change(smoothed_joystick_inputs_.axis4, prev_smoothed.axis4);
        smoothed_joystick_inputs_.axis5 = clamp_change(smoothed_joystick_inputs_.axis5, prev_smoothed.axis5);
        smoothed_joystick_inputs_.axis6 = clamp_change(smoothed_joystick_inputs_.axis6, prev_smoothed.axis6);
        smoothed_joystick_inputs_.axis7 = clamp_change(smoothed_joystick_inputs_.axis7, prev_smoothed.axis7);
        prev_smoothed = smoothed_joystick_inputs_;
        
        // Copy flags (don't smooth boolean values)
        smoothed_joystick_inputs_.emergency_stop = raw_cmd.emergency_stop;
        smoothed_joystick_inputs_.reset_pose = raw_cmd.reset_pose;
        
        // Use filtered joystick inputs for velocity calculation
        JoystickCommand cmd = smoothed_joystick_inputs_;
        
        // Debug: Show input filtering effect occasionally
        static int filter_debug_count = 0;
        filter_debug_count++;
        if (filter_debug_count % 100 == 0 && (std::abs(raw_cmd.axis0) > 0.01 || std::abs(raw_cmd.axis1) > 0.01 || std::abs(raw_cmd.axis2) > 0.01 ||
            std::abs(raw_cmd.axis3) > 0.01 || std::abs(raw_cmd.axis4) > 0.01 || std::abs(raw_cmd.axis6) > 0.01 ||
            std::abs(raw_cmd.axis7) > 0.01)) {
            std::cout << "INPUT FILTERING: Raw[" << raw_cmd.axis0 << "," << raw_cmd.axis1 << "," << raw_cmd.axis2 << "," << raw_cmd.axis3 << "," << raw_cmd.axis4 << "," << raw_cmd.axis5 << "," << raw_cmd.axis6 << "," << raw_cmd.axis7 << "]"
                      << " -> Filtered[" << cmd.axis0 << "," << cmd.axis1 << "," << cmd.axis2 << "," << cmd.axis3 << "," << cmd.axis4 << "," << cmd.axis5 << "," << cmd.axis6 << "," << cmd.axis7 << "]" << std::endl;
        }
        
        // Calculate target velocities from joystick input
        // NEW CONTROL MAPPING:
        // - D-pad controls robot translation (forward/back, left/right)  
        // - Left joystick controls end-effector orientation (pitch/yaw)
        // - Right joystick controls Z-movement and roll
        Eigen::Vector3d target_vel;
        target_vel[0] = cmd.axis7 * config_.max_linear_velocity;    // D-pad vertical -> forward/backward
        target_vel[1] = -cmd.axis6 * config_.max_linear_velocity;   // D-pad horizontal -> left/right (inverted)
        target_vel[2] = cmd.axis4 * config_.max_linear_velocity;    // Right stick vertical -> up/down
        
        Eigen::Vector3d target_angular_vel;
        target_angular_vel[0] = cmd.axis0 * config_.max_angular_velocity;  // Left stick horizontal -> roll (SWAPPED)
        target_angular_vel[1] = cmd.axis1 * config_.max_angular_velocity;  // Left stick vertical -> pitch  
        target_angular_vel[2] = cmd.axis3 * config_.max_angular_velocity;  // Right stick horizontal -> yaw (SWAPPED)
        
        // Apply extra aggressive smoothing for very small noisy inputs
        double smoothing_factor = config_.velocity_smoothing;
        if (target_vel.norm() < config_.noise_threshold) {
            smoothing_factor = config_.noise_smoothing;  // Much more aggressive smoothing for noise
        }
        
        // Apply adaptive exponential smoothing to velocities
        smoothed_velocity_ = (1.0 - smoothing_factor) * smoothed_velocity_ + 
                            smoothing_factor * target_vel;
        smoothed_angular_velocity_ = (1.0 - smoothing_factor) * smoothed_angular_velocity_ + 
                                   smoothing_factor * target_angular_vel;
        
        // Apply smooth deadzone to smoothed velocities (prevents discontinuities)
        for (int i = 0; i < 3; i++) {
            // Smooth linear deadzone using a cubic transition
            if (std::abs(smoothed_velocity_[i]) < config_.deadzone_linear) {
                double ratio = std::abs(smoothed_velocity_[i]) / config_.deadzone_linear;
                double smooth_factor = ratio * ratio * (3.0 - 2.0 * ratio); // Smooth cubic transition
                smoothed_velocity_[i] *= smooth_factor;
            }
            
            // Smooth angular deadzone using a cubic transition
            if (std::abs(smoothed_angular_velocity_[i]) < config_.deadzone_angular) {
                double ratio = std::abs(smoothed_angular_velocity_[i]) / config_.deadzone_angular;
                double smooth_factor = ratio * ratio * (3.0 - 2.0 * ratio); // Smooth cubic transition
                smoothed_angular_velocity_[i] *= smooth_factor;
            }
        }
        
        // Additional micro-motion suppression for ultra-small velocities
        for (int i = 0; i < 3; i++) {
            if (std::abs(smoothed_velocity_[i]) < 0.0001) {  // 0.1mm/s threshold
                smoothed_velocity_[i] *= 0.1;  // Heavily suppress micro-motions
            }
            if (std::abs(smoothed_angular_velocity_[i]) < 0.0001) {  // Very small angular threshold
                smoothed_angular_velocity_[i] *= 0.1;  // Heavily suppress micro-rotations
            }
        }
        
        // Update virtual target position using smoothed velocities with step size clamping
        Eigen::Vector3d desired_position_change = smoothed_velocity_ * dt;
        
        // Clamp position change to maximum step size to prevent discontinuities
        if (desired_position_change.norm() > config_.max_step_size) {
            double original_norm = desired_position_change.norm();
            desired_position_change = desired_position_change.normalized() * config_.max_step_size;
            static int clamp_count = 0;
            clamp_count++;
            if (clamp_count % 50 == 0) {  // Report every 50th clamping event
                std::cout << "STEP CLAMPING: Reduced step from " << original_norm 
                          << " to " << config_.max_step_size << " (factor " << (original_norm/config_.max_step_size) << ")" << std::endl;
            }
        }
        
        virtual_target_.translation() += desired_position_change;
        
        // Update virtual target orientation using smoothed angular velocities with step clamping
        if (smoothed_angular_velocity_.norm() > 1e-6) {
            double desired_angle = smoothed_angular_velocity_.norm() * dt;
            
            // Clamp angular change to maximum angular step size
            if (desired_angle > config_.max_angular_step) {
                static int angular_clamp_count = 0;
                angular_clamp_count++;
                if (angular_clamp_count % 50 == 0) {  // Report every 50th clamping event
                    std::cout << "ANGULAR STEP CLAMPING: Reduced angle from " << desired_angle 
                              << " to " << config_.max_angular_step << std::endl;
                }
                desired_angle = config_.max_angular_step;
            }
            
            Eigen::Vector3d axis = smoothed_angular_velocity_.normalized();
            Eigen::AngleAxisd delta_rot(desired_angle, axis);
            virtual_target_.linear() = virtual_target_.linear() * delta_rot.toRotationMatrix();
        }
        
        // Enforce workspace limits
        enforceWorkspaceLimits(virtual_target_);
    }
    
    void updateVirtualTargetDebug(double dt, int iteration) {
        // Get current joystick command
        JoystickCommand cmd;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            cmd = current_command_;
        }
        
        if (iteration <= 10) {
            std::cout << "DEBUG updateVirtualTarget: dt=" << dt << ", joystick=[" 
                      << cmd.axis0 << "," << cmd.axis1 << "," << cmd.axis2 << "," << cmd.axis3 << "," << cmd.axis4 << "," << cmd.axis5 << "," << cmd.axis6 << "," << cmd.axis7 << "]" << std::endl;
        }
        
        // Calculate target velocities from joystick input
        // NEW CONTROL MAPPING:
        // - D-pad controls robot translation (forward/back, left/right)  
        // - Left joystick controls end-effector orientation (pitch/yaw)
        // - Right joystick controls Z-movement and roll
        Eigen::Vector3d target_vel;
        target_vel[0] = cmd.axis7 * config_.max_linear_velocity;    // D-pad vertical -> forward/backward
        target_vel[1] = -cmd.axis6 * config_.max_linear_velocity;   // D-pad horizontal -> left/right (inverted)
        target_vel[2] = cmd.axis4 * config_.max_linear_velocity;    // Right stick vertical -> up/down
        
        Eigen::Vector3d target_angular_vel;
        target_angular_vel[0] = cmd.axis0 * config_.max_angular_velocity;  // Left stick horizontal -> roll (SWAPPED)
        target_angular_vel[1] = cmd.axis1 * config_.max_angular_velocity;  // Left stick vertical -> pitch  
        target_angular_vel[2] = cmd.axis3 * config_.max_angular_velocity;  // Right stick horizontal -> yaw (SWAPPED)
        
        if (iteration <= 10) {
            std::cout << "DEBUG: target_vel=[" << target_vel.transpose() << "]" << std::endl;
            std::cout << "DEBUG: smoothed_velocity_ before=[" << smoothed_velocity_.transpose() << "]" << std::endl;
        }
        
        // Apply exponential smoothing to velocities
        Eigen::Vector3d prev_smoothed_velocity = smoothed_velocity_;
        smoothed_velocity_ = (1.0 - config_.velocity_smoothing) * smoothed_velocity_ + 
                            config_.velocity_smoothing * target_vel;
        smoothed_angular_velocity_ = (1.0 - config_.velocity_smoothing) * smoothed_angular_velocity_ + 
                                   config_.velocity_smoothing * target_angular_vel;
        
        if (iteration <= 10) {
            std::cout << "DEBUG: smoothed_velocity_ after=[" << smoothed_velocity_.transpose() << "]" << std::endl;
        }
        
        // Apply deadzone to smoothed velocities
        for (int i = 0; i < 3; i++) {
            if (std::abs(smoothed_velocity_[i]) < config_.deadzone_linear) {
                smoothed_velocity_[i] = 0.0;
            }
            if (std::abs(smoothed_angular_velocity_[i]) < config_.deadzone_angular) {
                smoothed_angular_velocity_[i] = 0.0;
            }
        }
        
        if (iteration <= 10) {
            std::cout << "DEBUG: smoothed_velocity_ after deadzone=[" << smoothed_velocity_.transpose() << "]" << std::endl;
        }
        
        // Store previous position for debugging
        Eigen::Vector3d prev_position = virtual_target_.translation();
        
        // Update virtual target position using smoothed velocities
        virtual_target_.translation() += smoothed_velocity_ * dt;
        
        if (iteration <= 10) {
            std::cout << "DEBUG: position change=[" << (virtual_target_.translation() - prev_position).transpose() << "]" << std::endl;
            std::cout << "DEBUG: new position=[" << virtual_target_.translation().transpose() << "]" << std::endl;
        }
        
        // Update virtual target orientation using smoothed angular velocities
        if (smoothed_angular_velocity_.norm() > 1e-6) {
            double angle = smoothed_angular_velocity_.norm() * dt;
            Eigen::Vector3d axis = smoothed_angular_velocity_.normalized();
            Eigen::AngleAxisd delta_rot(angle, axis);
            virtual_target_.linear() = virtual_target_.linear() * delta_rot.toRotationMatrix();
            
            if (iteration <= 10) {
                std::cout << "DEBUG: Applied rotation, angle=" << angle << std::endl;
            }
        }
        
        // Enforce workspace limits
        Eigen::Vector3d pos_before_limits = virtual_target_.translation();
        enforceWorkspaceLimits(virtual_target_);
        
        if (iteration <= 10 && (virtual_target_.translation() - pos_before_limits).norm() > 1e-6) {
            std::cout << "DEBUG: Workspace limits applied, change=[" 
                      << (virtual_target_.translation() - pos_before_limits).transpose() << "]" << std::endl;
        }
    }
    
    franka::CartesianPose eigenToCartesianPose(const Eigen::Affine3d& pose) {
        std::array<double, 16> pose_array;
        Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = pose.matrix();
        return franka::CartesianPose(pose_array);
    }
    
    void teleoperation_loop(franka::Robot& robot) {
        std::cout << "Starting teleoperation loop at 100Hz..." << std::endl;
        
        const auto loop_duration = std::chrono::milliseconds(10); // 100Hz
        auto next_iteration_time = std::chrono::steady_clock::now();
        
        while (running_ && !emergency_stop_) {
            next_iteration_time += loop_duration;
            
            // Update virtual target based on joystick input
            updateVirtualTarget(0.01); // 10ms = 0.01s
            
            // Send target to robot through active motion generator
            try {
                franka::CartesianPose pose_command = eigenToCartesianPose(virtual_target_);
                
                // Here we would use robot.writeOnce(pose_command) if we had an active motion generator
                // For now, we'll implement this differently using the available API
                
            } catch (const franka::Exception& e) {
                std::cout << "Control exception in teleoperation loop: " << e.what() << std::endl;
                break;
            }
            
            // Maintain 100Hz timing
            std::this_thread::sleep_until(next_iteration_time);
        }
        
        std::cout << "Teleoperation loop finished." << std::endl;
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
            
            // Configure cartesian impedance for more responsive control
            robot.setCartesianImpedance({{1500, 1500, 1500, 150, 150, 150}});
            
            // Configure custom end-effector (0.7kg mass)
            std::array<double, 16> EE_T_K = {{1.0, 0.0, 0.0, 0.0,
                                              0.0, 1.0, 0.0, 0.0,
                                              0.0, 0.0, 1.0, 0.0,
                                              0.0, 0.0, 0.0, 1.0}};
            robot.setEE(EE_T_K);
            
            // Set end-effector mass and inertia
            double ee_mass = 0.7;  // 0.7kg custom end-effector
            std::array<double, 3> ee_com = {{0.0, 0.0, 0.05}};
            std::array<double, 9> ee_inertia = {{0.01, 0.0, 0.0,
                                                 0.0, 0.01, 0.0,
                                                 0.0, 0.0, 0.01}};
            robot.setLoad(ee_mass, ee_com, ee_inertia);
            
            std::cout << "Configured custom end-effector: mass=" << ee_mass << "kg" << std::endl;
            
            // Get initial pose and set as virtual target
            franka::RobotState initial_state = robot.readOnce();
            initial_pose_ = Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            virtual_target_ = initial_pose_;
            
            std::cout << "Initial pose set. Starting network thread..." << std::endl;
            std::thread network_thread(&TeleoperationController::networkThread, this);
            
            std::cout << "Starting cartesian impedance control with pose motion generator..." << std::endl;
            std::cout << "Press joystick B button for emergency stop." << std::endl;
            
            // Instead of using ActiveMotionGenerator (which we need to research more),
            // let's implement a simplified version using regular cartesian pose control
            this->runRobustTeleop(robot);
            
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
    
private:
    void runRobustTeleop(franka::Robot& robot) {
        std::cout << "DEBUG: Starting robust teleop with safe reset handling..." << std::endl;
        
        // Add error recovery and delay after joint motion (proven to work)
        try {
            std::cout << "DEBUG: Running automatic error recovery..." << std::endl;
            robot.automaticErrorRecovery();
            std::cout << "DEBUG: Error recovery completed." << std::endl;
        } catch (const franka::Exception& e) {
            std::cout << "DEBUG: Error recovery failed (this might be normal): " << e.what() << std::endl;
        }
        
        // Wait for robot to settle (proven to work)
        std::cout << "DEBUG: Waiting 2 seconds for robot to settle..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Reset smoothed velocities to ensure clean start
        smoothed_velocity_.setZero();
        smoothed_angular_velocity_.setZero();
        std::cout << "DEBUG: Reset smoothed velocities to zero" << std::endl;
        
        // Read current state for reference (proven to work)
        franka::RobotState reference_state = robot.readOnce();
        std::cout << "DEBUG: Reference pose obtained." << std::endl;
        
        // Initialize virtual target to current desired pose
        Eigen::Affine3d current_pose(Eigen::Matrix4d::Map(reference_state.O_T_EE_d.data()));
        virtual_target_ = current_pose;
        initial_pose_ = current_pose;
        std::cout << "DEBUG: Virtual target initialized to desired pose." << std::endl;
        std::cout << "  Position: [" << current_pose.translation().transpose() << "]" << std::endl;
        
        static int iteration_count = 0;
        iteration_count = 0;  // Reset counter
        
        // Flag for safe reset handling
        std::atomic<bool> reset_requested{false};
        
        // Robust teleop motion generator with safe reset handling
        auto robust_teleop_generator = [this, &iteration_count, &reset_requested](const franka::RobotState& robot_state, 
                                                                                 franka::Duration period) -> franka::CartesianPose {
            
            iteration_count++;
            double dt = period.toSec();
            
            // Only debug first 5 iterations to reduce spam
            if (iteration_count <= 5) {
                std::cout << "ROBUST: Iteration " << iteration_count << ", dt=" << dt << std::endl;
            }
            
            // Check for emergency stop
            if (emergency_stop_) {
                std::cout << "ROBUST: Emergency stop detected, finishing motion" << std::endl;
                return franka::MotionFinished(eigenToCartesianPose(virtual_target_));
            }
            
            // Update virtual target based on joystick input (only after stability period)
            if (iteration_count > 10 && dt > 0.0) {
                // Get current joystick command
                JoystickCommand cmd;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    cmd = current_command_;
                }
                
                // Handle reset pose safely - gradual return to initial pose instead of jump
                if (cmd.reset_pose && !reset_requested) {
                    std::cout << "ROBUST: Reset pose requested - gradual return to initial" << std::endl;
                    reset_requested = true;
                }
                
                // If reset is requested, gradually move back to initial pose
                if (reset_requested) {
                    Eigen::Vector3d to_initial = initial_pose_.translation() - virtual_target_.translation();
                    double distance = to_initial.norm();
                    
                    if (distance > 0.01) {  // 1cm threshold
                        // Move gradually towards initial pose
                        Eigen::Vector3d reset_velocity = to_initial.normalized() * 0.05;  // 5cm/s
                        virtual_target_.translation() += reset_velocity * dt;
                    } else {
                        // Close enough, reset complete
                        virtual_target_ = initial_pose_;
                        reset_requested = false;
                        smoothed_velocity_.setZero();
                        smoothed_angular_velocity_.setZero();
                        std::cout << "ROBUST: Reset complete" << std::endl;
                    }
                } else {
                    // Normal joystick control - only if not resetting
                    if (!cmd.reset_pose) {
                        // Store previous position for debugging (only occasionally)
                        Eigen::Vector3d prev_position = virtual_target_.translation();
                        
                        // Update virtual target using the working update method
                        updateVirtualTarget(dt);
                        
                        // Debug position changes only if significant or occasionally
                        double pos_change = (virtual_target_.translation() - prev_position).norm();
                        if (pos_change > 0.005 || (iteration_count % 1000 == 0)) {  // Every 1000 iterations (1 second)
                            std::cout << "ROBUST: Position change: " << pos_change << " m" << std::endl;
                        }
                    }
                }
            } else if (iteration_count <= 10) {
                if (iteration_count <= 5) {
                    std::cout << "ROBUST: Stability period, iteration " << iteration_count << std::endl;
                }
            }
            
            // Return virtual target as CartesianPose
            return eigenToCartesianPose(virtual_target_);
        };
        
        try {
            std::cout << "DEBUG: Starting robust teleop motion generator..." << std::endl;
            robot.control(robust_teleop_generator);
            std::cout << "DEBUG: Robust teleop motion generator finished normally." << std::endl;
        } catch (const franka::ControlException& e) {
            std::cout << "DEBUG: Robust teleop control exception: " << e.what() << std::endl;
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