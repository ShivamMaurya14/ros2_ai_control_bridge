#pragma once

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace ai_control_bridge_core {

/**
 * @brief Abstract base class for the AI Control Bridge.
 * 
 * Defines the interface that all controller bridge implementations must follow.
 * Bridges low-frequency AI policy outputs to high-frequency robot control commands.
 * 
 * This is designed as a ros2_control ControllerInterface plugin.
 * 
 * Thread Model:
 * - on_configure, on_activate, on_deactivate run in non-realtime thread
 * - update() runs in realtime control loop thread
 */
class ControllerBridge : public controller_interface::ControllerInterface {
public:
    virtual ~ControllerBridge() = default;

    /**
     * @brief Initialize controller from configuration (non-realtime).
     * Initialize topic subscriptions, parameters, etc.
     */
    virtual controller_interface::CallbackReturn on_init() override = 0;

    /**
     * @brief Configure controller (non-realtime).
     * Called when controller transitions to CONFIGURED state.
     */
    virtual controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override = 0;

    /**
     * @brief Activate controller (non-realtime).
     * Called when controller transitions to ACTIVE state.
     */
    virtual controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override = 0;

    /**
     * @brief Deactivate controller (non-realtime).
     * Called when controller transitions out of ACTIVE state.
     */
    virtual controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override = 0;

    /**
     * @brief Main control loop update (realtime).
     * Called at high frequency (~1 kHz) in realtime thread.
     * Must be deterministic and lock-free.
     * 
     * @param time Current time
     * @param period Time since last update
     * @return SUCCESS if update completed, FAILURE if error
     */
    virtual controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override = 0;

    /**
     * @brief Get the expected command interface configuration.
     * Specifies which command interfaces this controller needs.
     */
    virtual controller_interface::InterfaceConfiguration 
    command_interface_configuration() const override = 0;

    /**
     * @brief Get the expected state interface configuration.
     * Specifies which state interfaces this controller needs to read.
     */
    virtual controller_interface::InterfaceConfiguration 
    state_interface_configuration() const override = 0;
};

}  // namespace ai_control_bridge_core
