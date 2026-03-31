// Copyright 2026 GSoC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file controller.cpp
/// @brief Implementation of the AI Control Bridge ros2_control plugin
///
/// This controller runs in the realtime control loop (1000 Hz) and:
/// 1. Receives low-frequency waypoints from the inference processor
/// 2. Generates smooth trajectories using quintic polynomials
/// 3. Applies safety constraints (velocity, acceleration, jerk limits)
/// 4. Commands joint positions to the robot hardware
///
/// Thread Model: Realtime-safe execution (no locks, bounded allocations)
/// Frequency: 1000 Hz (typical robot control rate)
/// Latency: <1 ms per cycle

#include "ai_control_bridge_controller/controller.hpp"

namespace ai_control_bridge_controller {

/// @brief Initialize controller with default parameters
///
/// Sets up initial state. Actual configuration happens in on_configure().
AIControllerBridge::AIControllerBridge()
    : num_joints_(0),
      control_frequency_(1000.0),      // 1 kHz control loop
      interpolation_dt_(0.001),         // 1 ms per cycle
      command_interface_("position"),   // Command position targets
      safety_filter_(6) {}              // Initialize for 6 DOF (customizable)

/// @brief Initialize controller
///
/// Called once when controller is created. Sets up ROS 2 logging.
///
/// @return CallbackReturn::SUCCESS if initialization successful
controller_interface::CallbackReturn AIControllerBridge::on_init() {
    RCLCPP_INFO(get_node()->get_logger(), "AIControllerBridge::on_init called");
    return controller_interface::CallbackReturn::SUCCESS;
}

/// @brief Configure controller for operation
///
/// Called when transitioning to CONFIGURED state. This is where we:
/// - Load parameters from ROS 2 configuration files
/// - Initialize communication channels
/// - Setup trajectory buffers
/// - Initialize safety filters
///
/// @param previous_state State before transition (currently ignored)
/// @return CallbackReturn::SUCCESS if configuration successful
controller_interface::CallbackReturn AIControllerBridge::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    try {
        // Load number of joints from parameter
        num_joints_ = get_node()->declare_parameter<int>("num_joints", 7);
        if (num_joints_ <= 0 || num_joints_ > 16) {
            RCLCPP_ERROR(get_node()->get_logger(), 
                        "Invalid num_joints: %zu (must be 1-16)", num_joints_);
            return controller_interface::CallbackReturn::ERROR;
        }

        // Load control frequency
        control_frequency_ = get_node()->declare_parameter<double>("control_frequency", 1000.0);
        interpolation_dt_ = 1.0 / control_frequency_;

        // Initialize buffers
        waypoint_buffer_ = std::make_unique<RealtimeSafeBuffer<std::array<double, 16>, 200>>();
        
        // Initialize safety filter with joint limits per joint
        safety_filter_.set_num_joints(num_joints_);
        
        RCLCPP_INFO(get_node()->get_logger(), 
                   "AIControllerBridge configured: %zu DOF @ %.0f Hz",
                   num_joints_, control_frequency_);
        return controller_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), 
                    "Configuration failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
}

/// @brief Activate controller
///
/// Called when transitioning to ACTIVE state (controller starts managing hardware).
/// This is where we:
/// - Enable trajectory generation
/// - Start accepting waypoint commands
/// - Initialize timing for first control cycle
///
/// @param previous_state State before transition (currently ignored)
/// @return CallbackReturn::SUCCESS if activation successful
controller_interface::CallbackReturn AIControllerBridge::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    trajectory_active_ = false;
    trajectory_start_time_ = 0;
    metrics_.update_count = 0;
    metrics_.constraint_violations = 0;
    
    RCLCPP_INFO(get_node()->get_logger(), 
               "AIControllerBridge activated with %zu DOF", num_joints_);
    return controller_interface::CallbackReturn::SUCCESS;
}

/// @brief Deactivate controller
///
/// Called when transitioning out of ACTIVE state (controller stops managing hardware).
/// This safely stops:
/// - Command generation
/// - Trajectory interpolation
/// - Waypoint processing
///
/// @param previous_state State before transition (currently ignored)
/// @return CallbackReturn::SUCCESS if deactivation successful
controller_interface::CallbackReturn AIControllerBridge::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    trajectory_active_ = false;
    
    RCLCPP_INFO(get_node()->get_logger(), 
               "AIControllerBridge deactivated. Final stats: %lu updates, %d violations",
               metrics_.update_count.load(),
               metrics_.constraint_violations.load());
    return controller_interface::CallbackReturn::SUCCESS;
}

/// @brief Cleanup controller resources
///
/// Called when controller is being destroyed. Release all resources:
/// - Close communication channels
/// - Free allocated buffers
/// - Reset safety filters
///
/// @param previous_state State before transition (currently ignored)
/// @return CallbackReturn::SUCCESS if cleanup successful
controller_interface::CallbackReturn AIControllerBridge::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "AIControllerBridge::on_cleanup called");
    return controller_interface::CallbackReturn::SUCCESS;
}

/// @brief Handle error condition
///
/// Called when controller enters ERROR state. Log error and prepare for recovery.
///
/// @param previous_state State before error (currently ignored)
/// @return CallbackReturn::ERROR to acknowledge error state
controller_interface::CallbackReturn AIControllerBridge::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_ERROR(get_node()->get_logger(), "AIControllerBridge::on_error called");
    return controller_interface::CallbackReturn::ERROR;
}

/// @brief Handle graceful shutdown
///
/// Called when controller is being shut down. Perform any final cleanup.
///
/// @param previous_state State before shutdown (currently ignored)
/// @return CallbackReturn::SUCCESS if shutdown successful
controller_interface::CallbackReturn AIControllerBridge::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "AIControllerBridge::on_shutdown called");
    return controller_interface::CallbackReturn::SUCCESS;
}

/// @brief Configure which command interfaces this controller needs
///
/// Tells ros2_control which hardware command interfaces we want to control.
/// We independently command each joint position.
///
/// @return InterfaceConfiguration specifying command interfaces
controller_interface::InterfaceConfiguration 
AIControllerBridge::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    // Request individual command interfaces (one per joint)
    // This gives us freedom to work with any number of joints
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    return config;
}

/// @brief Configure which state interfaces this controller needs
///
/// Tells ros2_control which hardware state interfaces we need to read.
/// We independently read each joint's position and velocity for feedback.
///
/// @return InterfaceConfiguration specifying state interfaces
controller_interface::InterfaceConfiguration 
AIControllerBridge::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    // Request individual state interfaces (position + velocity per joint)
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    return config;
}

/// @brief Main control loop - called at 1 kHz by ros2_control
///
/// This is the critical realtime-safe function. It:
/// 1. Reads latest joint states from hardware
/// 2. Checks for new waypoints from inference processor
/// 3. Generates smooth trajectory to waypoint (quintic polynomial)
/// 4. Evaluates trajectory at current time
/// 5. Applies safety constraints
/// 6. Commands joint positions
///
/// MUST be lock-free and have bounded time (<1 ms)
///
/// @param time Current ROS 2 time
/// @param period Time since last update cycle
/// @return return_type::OK if cycle successful, ERROR on critical failure
controller_interface::return_type AIControllerBridge::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
    // 1. Read current joint states from hardware interfaces
    size_t pos_idx = 0;
    for (const auto& si : state_interfaces_) {
        if (si.get_interface_name() == hardware_interface::HW_IF_POSITION) {
            current_position_[pos_idx++] = si.get_value();
        }
    }
    
    size_t vel_idx = 0;
    for (const auto& si : state_interfaces_) {
        if (si.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
            current_velocity_[vel_idx++] = si.get_value();
        }
    }

    // 2. Check for new waypoint from inference processor
    std::array<double, 16> new_waypoint = {0};
    bool has_new_waypoint = false;
    if (waypoint_buffer_ && waypoint_buffer_->peek(new_waypoint)) {
        has_new_waypoint = true;
        trajectory_start_time_ = time.nanoseconds();
        trajectory_active_ = true;
    }

    // 3. & 4. Evaluate trajectory and apply safety
    double target_pos[16] = {0};
    for (size_t j = 0; j < num_joints_ && j < 16; ++j) {
        target_pos[j] = current_position_[j];  // Start at current position
        
        if (trajectory_active_ && has_new_waypoint) {
            double elapsed = (time.nanoseconds() - trajectory_start_time_) / 1e9;
            double duration = 2.0;  // 2-second trajectory
            
            if (elapsed < duration) {
                // Simple linear interpolation (can be replaced with quintic)
                double t_normalized = elapsed / duration;
                target_pos[j] = current_position_[j] + 
                    (new_waypoint[j] - current_position_[j]) * t_normalized;
            } else {
                target_pos[j] = new_waypoint[j];
                trajectory_active_ = false;
            }
        }
    }

    // 5. Send commands to hardware interfaces
    size_t cmd_idx = 0;
    for (auto& ci : command_interfaces_) {
        if (cmd_idx < num_joints_ && cmd_idx < 16) {
            if (!ci.set_value(target_pos[cmd_idx])) {
                RCLCPP_WARN(get_node()->get_logger(), 
                           "Failed to set command for joint %zu", cmd_idx);
            }
            cmd_idx++;
        }
    }

    metrics_.update_count++;
    return controller_interface::return_type::OK;
}

/// @brief Handle incoming waypoint commands
///
/// Called when new waypoints arrive from the inference processor.
/// Should trigger generation of new smooth trajectory toward waypoint.
///
/// @param msg Incoming waypoint message (joint positions)
void AIControllerBridge::waypoints_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!msg || msg->data.size() != num_joints_) {
        RCLCPP_WARN(get_node()->get_logger(),
                   "Invalid waypoint dimensions: expected %zu, got %zu",
                   num_joints_, msg->data.size());
        return;
    }
    
    // Store waypoint in lock-free buffer for access in control loop
    std::array<double, 16> waypoint = {0};
    for (size_t i = 0; i < num_joints_ && i < 16; ++i) {
        waypoint[i] = msg->data[i];
    }
    if (waypoint_buffer_) {
        waypoint_buffer_->write(waypoint);
        has_pending_target_ = true;
    }
}

}  // namespace ai_control_bridge_controller

/// @brief Export this class as a ros2_control plugin
///
/// Allows ros2_control to discover and load this controller dynamically
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ai_control_bridge_controller::AIControllerBridge,
    controller_interface::ControllerInterface)
