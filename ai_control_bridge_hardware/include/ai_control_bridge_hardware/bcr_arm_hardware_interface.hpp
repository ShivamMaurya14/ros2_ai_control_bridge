// Copyright 2026 GSoC Contributors
// Licensed under the Apache License, Version 2.0

/**
 * @file bcr_arm_hardware_interface.hpp
 * @brief Hardware interface adapter for BCR ARM real robot
 * 
 * This provides the bridge between ros2_control framework and actual BCR ARM hardware.
 * 
 * Responsible for:
 * - Communicating with robot controller (EtherCAT/CAN/etc)
 * - Reading joint encoder values
 * - Writing joint command values
 * - Managing connection state and error handling
 * 
 * Template implementation - customize for your specific hardware!
 */

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <cstdint>


namespace ai_control_bridge_hardware {

/**
 * @brief Hardware interface to BCR ARM real robot
 * 
 * Communicates with physical robot through hardware protocol:
 * - EtherCAT (common for industrial arms)
 * - CAN bus
 * - Custom USB/Ethernet protocol
 */
class BCRARMHardwareInterface {
public:
    /// Joint state from hardware
    struct JointState {
        double position;      ///< Joint angle (radians)
        double velocity;      ///< Joint velocity (rad/s)
        double effort;        ///< Joint torque (Nm)
        uint32_t timestamp;   ///< When this state was sampled
    };
    
    /// Command to hardware
    struct JointCommand {
        double position;      ///< Desired joint angle (radians)
        double velocity;      ///< Optional velocity feedforward (rad/s)
        double effort;        ///< Optional torque feedforward (Nm)
    };
    
    explicit BCRARMHardwareInterface(int num_joints = 7);
    virtual ~BCRARMHardwareInterface();
    
    /// Initialize connection to hardware
    /// @param config_file Path to configuration YAML
    /// @return true if connection successful
    bool initialize(const std::string& config_file);
    
    /// Start communication
    bool start();
    
    /// Stop communication gracefully
    bool stop();
    
    /// Is hardware connected?
    bool is_connected() const;
    
    /// Read current joint states from hardware
    /// @param states Output: current state of each joint
    /// @return true if read successful
    bool read_joint_states(std::vector<JointState>& states);
    
    /// Write command to joint
    /// @param joint_idx Joint index (0 to num_joints-1)
    /// @param cmd Command to apply
    /// @return true if write successful
    bool write_joint_command(size_t joint_idx, const JointCommand& cmd);
    
    /// Emergency stop
    bool emergency_stop();
    
    /// Get number of joints
    int get_num_joints() const { return num_joints_; }
    
    /// Get last error message
    std::string get_error_message() const { return error_message_; }
    
private:
    int num_joints_;
    bool connected_;
    std::string error_message_;
    
    // Hardware-specific data (customize for your setup)
    struct HardwareState {
        std::vector<JointState> joint_states;
        std::vector<JointCommand> joint_commands;
        uint64_t cycle_count;
    };
    
    std::unique_ptr<HardwareState> hw_state_;
    
    /// Platform-specific connection logic
    /// CUSTOMIZE THIS for your robot!
    bool connect_ethercat(const std::string& interface);
    bool connect_can(const std::string& bus);
    bool connect_custom(const std::string& config);
};

}  // namespace ai_control_bridge_hardware
