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
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <kinematics.hpp>
#include <Eigen/Dense>


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
    
    // Ruckig members - these need to persist across control loop calls
    const int DOFS = 7;
    ruckig::Ruckig<7> ruckig_generator_{0.001}; // 1 ms update rate
    ruckig::InputParameter<7> ruckig_input_;
    ruckig::OutputParameter<7> ruckig_output_;
    
    // Control timing
    double control_time_{0.0};
    bool ruckig_initialized_{false};
    
    // Set kinematic limits for Ruckig (adjust these based on your robot's capabilities and safety)
    void initRuckigLimits() {
        for (size_t i = 0; i < DOFS; ++i) {
            ruckig_input_.max_velocity[i] = 0.5;       // rad/s - Conservative
            ruckig_input_.max_acceleration[i] = 1.0;   // rad/s^2 - Conservative 
            ruckig_input_.max_jerk[i] = 10.0;          // rad/s^3 - Conservative
        }
        // Don't initialize positions here - will be done in first control cycle
    }
    
    void initializeRuckig(const franka::RobotState& robot_state) {
        // Initialize current state from robot DESIRED state (not actual)
        std::cout << "Initializing Ruckig with robot desired state..." << std::endl;
        
        for (size_t i = 0; i < DOFS; ++i) {
            ruckig_input_.current_position[i] = robot_state.q_d[i];  // Use DESIRED positions
            ruckig_input_.current_velocity[i] = 0.0;  // Start with zero velocity
            ruckig_input_.current_acceleration[i] = 0.0; // Start with zero acceleration
            
            // Initialize targets to current desired state for smooth startup
            ruckig_input_.target_position[i] = robot_state.q_d[i];
            ruckig_input_.target_velocity[i] = 0.0;
            ruckig_input_.target_acceleration[i] = 0.0;
            
            std::cout << "Joint " << i << ": pos=" << robot_state.q_d[i] 
                     << " target=" << ruckig_input_.target_position[i] << std::endl;
        }
        ruckig_initialized_ = true;
        std::cout << "Ruckig initialized successfully" << std::endl;
    }
    
public:
    ModernFrankaController() {
        setupNetworking();
        initRuckigLimits(); // Call this in the constructor
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
            
            // Reset control timing for new session
            control_time_ = 0.0;
            ruckig_initialized_ = false;
            
            // --- REPLACING CARTESIAN VELOCITY CONTROL WITH JOINT POSITION CONTROL ---
            // Now, your callback will return franka::JointPositions after IK and Ruckig.
            auto callback_joint_position_control =
                [this](const franka::RobotState& robot_state,
                       franka::Duration period) -> franka::JointPositions {

                // Update control time
                control_time_ += period.toSec();
                
                // Initialize Ruckig on first call with actual robot state
                if (control_time_ == 0.0 || !ruckig_initialized_) {
                    initializeRuckig(robot_state);
                }

                // Check for emergency stop
                if (emergency_stop_) {
                    std::cout << "Emergency stop activated!" << std::endl;
                    // Return current position to hold robot when stopped
                    franka::JointPositions stop_positions(robot_state.q_d);
                    return franka::MotionFinished(stop_positions);
                }

                // 1. Get current joystick command (Cartesian velocities)
                JoystickCommand cmd;
                {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    cmd = current_command_;
                }

                // Convert joystick commands to target Cartesian velocities with deadband
                Eigen::Matrix<double, 6, 1> desired_cartesian_velocities;
                const double deadband = 0.02; // 2% deadband to prevent small noise
                
                auto apply_deadband = [deadband](double value) {
                    return (std::abs(value) < deadband) ? 0.0 : value;
                };
                
                desired_cartesian_velocities <<
                    apply_deadband(cmd.linear_y) * MAX_LINEAR_VEL,  // X (forward/backward)
                    apply_deadband(-cmd.linear_x) * MAX_LINEAR_VEL, // Y (left/right)
                    apply_deadband(cmd.linear_z) * MAX_LINEAR_VEL,  // Z (up/down)
                    apply_deadband(cmd.angular_x) * MAX_ANGULAR_VEL,
                    apply_deadband(cmd.angular_y) * MAX_ANGULAR_VEL,
                    apply_deadband(cmd.angular_z) * MAX_ANGULAR_VEL;

                // Check if all velocities are essentially zero
                bool is_stationary = desired_cartesian_velocities.norm() < 1e-6;

                // If stationary, just return current target (no updates)
                if (is_stationary) {
                    std::array<double, 7> current_targets;
                    for (size_t i = 0; i < DOFS; ++i) {
                        current_targets[i] = ruckig_input_.target_position[i];
                    }
                    return franka::JointPositions(current_targets);
                }

                // Only update targets when there's significant motion command
                static int update_counter = 0;
                update_counter++;
                
                if (update_counter % 10 == 0) { // Only update every 10th cycle to reduce discontinuities
                    std::cout << "Updating targets... cart_vel_norm=" << desired_cartesian_velocities.norm() << std::endl;
                    
                    // --- 2. IK LAYER (using frankx::Kinematics) ---
                    Eigen::Matrix<double, 7, 1> q_current_eigen = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q_d.data());
                    
                    // Calculate IK
                    Eigen::Matrix<double, 6, 7> J = frankx::Kinematics::jacobian(q_current_eigen);
                    Eigen::Matrix<double, 7, 6> J_inv = frankx::Kinematics::pseudoinverse(J);
                    Eigen::Matrix<double, 7, 1> desired_joint_velocities_ik = J_inv * desired_cartesian_velocities;

                    // Apply very conservative velocity limits
                    for (int i = 0; i < 7; i++) {
                        desired_joint_velocities_ik[i] = std::max(-0.1, std::min(0.1, desired_joint_velocities_ik[i]));
                    }

                    // Update targets with small increments (like frankx joint motion generator)
                    for (size_t i = 0; i < DOFS; ++i) {
                        // Very small position increment for continuous motion
                        double small_increment = desired_joint_velocities_ik[i] * 0.01; // 10ms worth
                        ruckig_input_.target_position[i] = ruckig_input_.current_position[i] + small_increment;
                        ruckig_input_.target_velocity[i] = 0.0; // Always target zero velocity
                        ruckig_input_.target_acceleration[i] = 0.0;
                        
                        if (i == 0 && std::abs(small_increment) > 1e-6) {
                            std::cout << "Joint 0: increment=" << small_increment 
                                     << " new_target=" << ruckig_input_.target_position[i] << std::endl;
                        }
                    }
                }

                // --- 3. RUCKIG TRAJECTORY GENERATION LAYER ---
                // Handle multiple steps per control cycle (like frankx)
                const int steps = std::max<int>(period.toMSec(), 1);
                ruckig::Result result;
                std::array<double, 7> joint_positions;
                
                for (int step = 0; step < steps; step++) {
                    // Calculate the next trajectory point
                    result = ruckig_generator_.update(ruckig_input_, ruckig_output_);
                    joint_positions = ruckig_output_.new_position;
                    
                    if (result == ruckig::Result::Finished) {
                        std::cout << "Ruckig finished - using target position" << std::endl;
                        joint_positions = ruckig_input_.target_position;
                        return franka::JointPositions(joint_positions);
                        
                    } else if (result == ruckig::Result::Error) {
                        std::cerr << "Ruckig error - holding current position" << std::endl;
                        // Print debug info
                        std::cout << "Current pos[0]=" << ruckig_input_.current_position[0] 
                                 << " vel[0]=" << ruckig_input_.current_velocity[0]
                                 << " target_pos[0]=" << ruckig_input_.target_position[0] << std::endl;
                        return franka::JointPositions(robot_state.q_d);
                    }
                    
                    // CRITICAL: Pass output to input for continuity (frankx pattern)
                    ruckig_output_.pass_to_input(ruckig_input_);
                }

                return franka::JointPositions(joint_positions);
            }; // End of callback_joint_position_control lambda

            std::cout << "Joystick control active! Use joystick to move robot." << std::endl;
            
            // Control loop with automatic restart after errors
            while (!emergency_stop_) {
                try {
                    // Start external Joint Position control loop
                    auto active_control = robot.startJointPositionControl(
                        research_interface::robot::Move::ControllerMode::kJointImpedance);
                    
                    bool motion_finished = false;
                    while (!motion_finished && !emergency_stop_) {
                        auto read_once_return = active_control->readOnce();
                        auto robot_state = read_once_return.first;
                        auto duration = read_once_return.second;
                        
                        // Call your joint position control callback
                        auto joint_positions_command = callback_joint_position_control(robot_state, duration);
                        motion_finished = joint_positions_command.motion_finished;
                        
                        // Write the JointPosition command
                        active_control->writeOnce(joint_positions_command);
                    }
                    
                    if (motion_finished) {
                        std::cout << "Motion finished normally." << std::endl;
                        break;
                    }
                    
                } catch (const franka::ControlException& e) {
                    std::cout << "Control exception: " << e.what() << std::endl;
                    std::cout << "Running error recovery..." << std::endl;
                    robot.automaticErrorRecovery();
                    
                    // Reset Ruckig state for clean restart
                    control_time_ = 0.0;
                    ruckig_initialized_ = false;
                    
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