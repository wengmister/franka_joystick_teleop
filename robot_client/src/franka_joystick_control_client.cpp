// Trajectory-Based Cartesian Teleoperation - Proper acceleration control
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <chrono>
#include <deque>
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
    double axis0 = 0.0, axis1 = 0.0, axis2 = 0.0, axis3 = 0.0;
    double axis4 = 0.0, axis5 = 0.0, axis6 = 0.0, axis7 = 0.0;
    bool emergency_stop = false;
    bool reset_pose = false;
};

// Simple trajectory point with guaranteed smooth derivatives
struct TrajectoryPoint {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;
    double time;
};

class TrajectoryCartesianController {
private:
    std::atomic<bool> running_{true};
    std::atomic<bool> emergency_stop_{false};
    JoystickCommand current_command_;
    std::mutex command_mutex_;
    
    int server_socket_;
    const int PORT = 8888;
    
    // Trajectory parameters - designed to prevent acceleration discontinuities
    struct TrajParams {
        double max_velocity = 0.8;           // 8cm/s
        double max_acceleration = 0.05;       // 5cm/s² - conservative
        double max_jerk = 0.1;              // 10cm/s³ - jerk limiting
        double max_angular_velocity = 0.3;    // 0.3 rad/s
        double max_angular_acceleration = 0.15; // 0.15 rad/s²
        double max_angular_jerk = 0.2;        // 0.2 rad/s³
        
        // Input processing
        double input_filter_freq = 100.0;     // 100Hz low-pass filter
        double deadzone_linear = 0.005;
        double deadzone_angular = 0.01;
    } params_;
    
    // Current trajectory state
    TrajectoryPoint current_point_;
    TrajectoryPoint target_point_;
    
    // Input filtering
    Eigen::Vector3d filtered_linear_input_{0, 0, 0};
    Eigen::Vector3d filtered_angular_input_{0, 0, 0};
    
    // Trajectory history for smooth derivatives
    std::deque<TrajectoryPoint> trajectory_history_;
    
    // Initial pose
    Eigen::Affine3d initial_pose_;
    
public:
    TrajectoryCartesianController() {
        setupNetworking();
        initializeTrajectoryPoint(current_point_);
        initializeTrajectoryPoint(target_point_);
    }
    
    ~TrajectoryCartesianController() {
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
                buffer[bytes_received] = '\0';
                
                JoystickCommand cmd;
                int parsed_count = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %d %d",
                          &cmd.axis0, &cmd.axis1, &cmd.axis2, &cmd.axis3, &cmd.axis4, &cmd.axis5, &cmd.axis6, &cmd.axis7,
                          (int*)&cmd.emergency_stop, (int*)&cmd.reset_pose);
                
                if (parsed_count == 10) {
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
    
private:
    void initializeTrajectoryPoint(TrajectoryPoint& point) {
        point.position.setZero();
        point.velocity.setZero();
        point.acceleration.setZero();
        point.orientation.setIdentity();
        point.angular_velocity.setZero();
        point.angular_acceleration.setZero();
        point.time = 0.0;
    }
    
    void filterInputs(const JoystickCommand& cmd, double dt) {
        // Convert joystick to desired velocities
        Eigen::Vector3d desired_linear;
        desired_linear[0] = cmd.axis7 * params_.max_velocity;
        desired_linear[1] = -cmd.axis6 * params_.max_velocity;
        desired_linear[2] = cmd.axis4 * params_.max_velocity;
        
        Eigen::Vector3d desired_angular;
        desired_angular[0] = cmd.axis0 * params_.max_angular_velocity;
        desired_angular[1] = cmd.axis1 * params_.max_angular_velocity;
        desired_angular[2] = cmd.axis3 * params_.max_angular_velocity;
        
        // Apply deadzone (but don't force to zero - let filter handle it)
        for (int i = 0; i < 3; i++) {
            if (std::abs(desired_linear[i]) < params_.deadzone_linear) {
                desired_linear[i] = 0.0;
            }
            if (std::abs(desired_angular[i]) < params_.deadzone_angular) {
                desired_angular[i] = 0.0;
            }
        }
        
        // Low-pass filter with gentle convergence
        double alpha = params_.input_filter_freq * dt / (1.0 + params_.input_filter_freq * dt);
        
        // Apply filtering normally - no hard zeroing
        filtered_linear_input_ = (1.0 - alpha) * filtered_linear_input_ + alpha * desired_linear;
        filtered_angular_input_ = (1.0 - alpha) * filtered_angular_input_ + alpha * desired_angular;
    }
    
    TrajectoryPoint generateNextTrajectoryPoint(double dt) {
        TrajectoryPoint next_point = current_point_;
        next_point.time += dt;
        
        // Generate smooth trajectory with gentle damping (no hard zeroing)
        for (int i = 0; i < 3; i++) {
            // Linear motion with gentle convergence
            double desired_vel = filtered_linear_input_[i];
            double vel_error = desired_vel - current_point_.velocity[i];
            
            // Apply gentle damping when input is very small
            double damping_factor = 1.0;
            if (std::abs(desired_vel) < params_.deadzone_linear) {
                // Gentle exponential decay instead of hard zeroing
                damping_factor = 0.99;  // Gentle 1% reduction per iteration
            }
            
            // Limit acceleration based on velocity error
            double desired_accel = vel_error / dt;
            double accel_error = desired_accel - current_point_.acceleration[i];
            
            // Limit jerk
            double jerk = std::max(-params_.max_jerk, 
                         std::min(params_.max_jerk, accel_error / dt));
            
            // Update acceleration with jerk limit
            next_point.acceleration[i] = current_point_.acceleration[i] + jerk * dt;
            
            // Apply gentle damping to acceleration when no input
            if (std::abs(desired_vel) < params_.deadzone_linear) {
                next_point.acceleration[i] *= damping_factor;
            }
            
            // Limit acceleration magnitude
            next_point.acceleration[i] = std::max(-params_.max_acceleration,
                                        std::min(params_.max_acceleration, next_point.acceleration[i]));
            
            // Update velocity with acceleration
            next_point.velocity[i] = current_point_.velocity[i] + next_point.acceleration[i] * dt;
            
            // Apply gentle damping to velocity when no input
            if (std::abs(desired_vel) < params_.deadzone_linear) {
                next_point.velocity[i] *= damping_factor;
            }
            
            // Limit velocity magnitude
            next_point.velocity[i] = std::max(-params_.max_velocity,
                                    std::min(params_.max_velocity, next_point.velocity[i]));
            
            // Update position with velocity
            next_point.position[i] = current_point_.position[i] + next_point.velocity[i] * dt;
            
            // Angular motion with same gentle approach
            double desired_angular_vel = filtered_angular_input_[i];
            double angular_vel_error = desired_angular_vel - current_point_.angular_velocity[i];
            
            // Apply gentle damping for angular motion
            double angular_damping_factor = 1.0;
            if (std::abs(desired_angular_vel) < params_.deadzone_angular) {
                angular_damping_factor = 0.98;  // Slightly faster damping
            }
            
            double desired_angular_accel = angular_vel_error / dt;
            double angular_accel_error = desired_angular_accel - current_point_.angular_acceleration[i];
            
            double angular_jerk = std::max(-params_.max_angular_jerk,
                                 std::min(params_.max_angular_jerk, angular_accel_error / dt));
            
            next_point.angular_acceleration[i] = current_point_.angular_acceleration[i] + angular_jerk * dt;
            
            // Apply gentle damping to angular acceleration
            if (std::abs(desired_angular_vel) < params_.deadzone_angular) {
                next_point.angular_acceleration[i] *= angular_damping_factor;
            }
            
            next_point.angular_acceleration[i] = std::max(-params_.max_angular_acceleration,
                                                std::min(params_.max_angular_acceleration, next_point.angular_acceleration[i]));
            
            next_point.angular_velocity[i] = current_point_.angular_velocity[i] + next_point.angular_acceleration[i] * dt;
            
            // Apply gentle damping to angular velocity
            if (std::abs(desired_angular_vel) < params_.deadzone_angular) {
                next_point.angular_velocity[i] *= angular_damping_factor;
            }
            
            next_point.angular_velocity[i] = std::max(-params_.max_angular_velocity,
                                            std::min(params_.max_angular_velocity, next_point.angular_velocity[i]));
        }
        
        // Update orientation using angular velocity (no threshold check)
        if (next_point.angular_velocity.norm() > 0.0) {
            double angle = next_point.angular_velocity.norm() * dt;
            if (angle > 1e-8) {  // Only very tiny threshold to avoid numerical issues
                Eigen::Vector3d axis = next_point.angular_velocity.normalized();
                Eigen::Quaterniond delta_quat(Eigen::AngleAxisd(angle, axis));
                next_point.orientation = current_point_.orientation * delta_quat;
                next_point.orientation.normalize();
            } else {
                next_point.orientation = current_point_.orientation;
            }
        } else {
            next_point.orientation = current_point_.orientation;
        }
        
        // Workspace enforcement with smooth boundaries
        enforceWorkspaceLimits(next_point);
        
        return next_point;
    }
    
    void enforceWorkspaceLimits(TrajectoryPoint& point) {
        Eigen::Vector3d relative_pos = point.position - initial_pose_.translation();
        Eigen::Vector3d workspace_min{-0.25, -0.25, -0.25};
        Eigen::Vector3d workspace_max{0.25, 0.25, 0.25};
        
        bool hit_limit = false;
        for (int i = 0; i < 3; i++) {
            if (relative_pos[i] < workspace_min[i]) {
                relative_pos[i] = workspace_min[i];
                point.velocity[i] = std::max(0.0, point.velocity[i]); // Only allow movement away from limit
                point.acceleration[i] = 0.0;
                hit_limit = true;
            } else if (relative_pos[i] > workspace_max[i]) {
                relative_pos[i] = workspace_max[i];
                point.velocity[i] = std::min(0.0, point.velocity[i]); // Only allow movement away from limit
                point.acceleration[i] = 0.0;
                hit_limit = true;
            }
        }
        
        if (hit_limit) {
            point.position = initial_pose_.translation() + relative_pos;
            static int limit_warn_count = 0;
            limit_warn_count++;
            if (limit_warn_count % 1000 == 0) {
                std::cout << "Workspace limit enforced (smooth)" << std::endl;
            }
        }
    }
    
    franka::CartesianPose trajectoryPointToCartesianPose(const TrajectoryPoint& point) {
        Eigen::Affine3d pose;
        pose.translation() = point.position;
        pose.linear() = point.orientation.toRotationMatrix();
        
        std::array<double, 16> pose_array;
        Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = pose.matrix();
        return franka::CartesianPose(pose_array);
    }
    
public:
    void run(const std::string& robot_ip) {
        try {
            std::cout << "Connecting to robot at " << robot_ip << std::endl;
            franka::Robot robot(robot_ip);
            setDefaultBehavior(robot);
            
            // Move to initial configuration
            std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            MotionGenerator motion_generator(0.5, q_goal);
            std::cout << "WARNING: This example will move the robot!" << std::endl
                      << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(motion_generator);
            
            // Configure robot for smooth operation
            robot.setCollisionBehavior(
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}}, {{100.0, 100.0, 80.0, 80.0, 80.0, 80.0, 60.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}},
                {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}}, {{80.0, 80.0, 80.0, 80.0, 80.0, 80.0}});
            
            // Lower impedance for smoother motion
            robot.setCartesianImpedance({{500, 500, 500, 50, 50, 50}});
            
            // Initialize trajectory from current pose
            franka::RobotState state = robot.readOnce();
            initial_pose_ = Eigen::Affine3d(Eigen::Matrix4d::Map(state.O_T_EE_d.data()));
            
            current_point_.position = initial_pose_.translation();
            current_point_.orientation = Eigen::Quaterniond(initial_pose_.rotation());
            
            std::thread network_thread(&TrajectoryCartesianController::networkThread, this);
            
            this->runTrajectoryCartesianTeleop(robot);
            
            running_ = false;
            network_thread.join();
            
        } catch (const franka::Exception& e) {
            std::cout << "Franka exception: " << e.what() << std::endl;
            running_ = false;
        }
    }
    
private:
    void runTrajectoryCartesianTeleop(franka::Robot& robot) {
        std::cout << "Starting trajectory-based Cartesian teleoperation..." << std::endl;
        
        try {
            robot.automaticErrorRecovery();
        } catch (const franka::Exception& e) {
            std::cout << "Error recovery: " << e.what() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        int iteration_count = 0;
        
        // Main trajectory generator. This is the lambda function that will be called by the robot control loop.
        // It generates the next Cartesian pose based on joystick input and current trajectory state.
        auto trajectory_generator = [this, iteration_count = 0]
                                   (const franka::RobotState& robot_state, franka::Duration period) mutable -> franka::CartesianPose {
            
            iteration_count++;
            double dt = period.toSec();
            
            if (emergency_stop_) {
                return franka::MotionFinished(trajectoryPointToCartesianPose(current_point_));
            }
            
            // Get and filter joystick input
            JoystickCommand cmd;
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                cmd = current_command_;
            }
            
            // Only start control after initialization
            if (iteration_count > 10 && dt > 0.0 && dt < 0.01) {
                filterInputs(cmd, dt);
                current_point_ = generateNextTrajectoryPoint(dt);
                
                // Debug output with gentler stability monitoring
                if (iteration_count % 2000 == 0) {
                    bool is_nearly_stable = (current_point_.velocity.norm() < 1e-3 && 
                                           current_point_.acceleration.norm() < 1e-3 &&
                                           current_point_.angular_velocity.norm() < 1e-3 &&
                                           current_point_.angular_acceleration.norm() < 1e-3);
                    
                    std::cout << "Traj: pos=[" << current_point_.position.x() << ", "
                              << current_point_.position.y() << ", " << current_point_.position.z() << "]"
                              << " vel=" << current_point_.velocity.norm()
                              << " accel=" << current_point_.acceleration.norm() 
                              << (is_nearly_stable ? " [NEARLY_STABLE]" : " [MOVING]") << std::endl;
                }
            }
            
            return trajectoryPointToCartesianPose(current_point_);
        };
        
        try {
            robot.control(trajectory_generator);
            std::cout << "Trajectory control finished normally." << std::endl;
        } catch (const franka::ControlException& e) {
            std::cout << "Trajectory control exception: " << e.what() << std::endl;
            std::cout << "Final state: pos=[" << current_point_.position.x() << ", "
                      << current_point_.position.y() << ", " << current_point_.position.z() << "]"
                      << " vel=" << current_point_.velocity.norm()
                      << " accel=" << current_point_.acceleration.norm() << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    
    try {
        TrajectoryCartesianController controller;
        controller.run(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}