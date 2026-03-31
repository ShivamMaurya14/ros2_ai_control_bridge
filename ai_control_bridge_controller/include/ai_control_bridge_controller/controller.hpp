#pragma once

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ai_control_bridge_controller/buffer.hpp"
#include "ai_control_bridge_controller/trajectory.hpp"
#include "ai_control_bridge_controller/safety.hpp"
#include "ai_control_bridge_core/postprocessor_core.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace ai_control_bridge_controller {

/**
 * @class AIControllerBridge
 * @brief Real-time control bridge for AI policy execution on any robot.
 * 
 * @section overview Overview
 * 
 * The AIControllerBridge is the main ros2_control plugin that connects learned
 * AI policies to hardware-level robot control. It addresses the fundamental
 * challenge: reconciling **low-frequency AI inference** (50 Hz) with 
 * **high-frequency hardware control** (1000 Hz).
 * 
 * @section architecture Architecture
 * 
 * @verbatim
 * Inference Thread (~50 Hz)      Control Thread (1000 Hz)
 * ┌─────────────────────────┐    ┌────────────────────────────┐
 * │ Run Neural Network      │    │ 1. Read Joint States       │
 * │ Generate Waypoint       │    │ 2. Check for New Waypoint  │
 * │ Publish to /waypoints   │    │ 3. Generate Trajectory     │
 * └────────────┬────────────┘    │ 4. Apply Safety Filters    │
 *              │                 │ 5. Send Commands           │
 *              │ waypoint        │ 6. Publish Debug Info      │
 *              │ (lock-free)     └────────────────────────────┘
 *              │                            ↑
 *              └────────────────────────────┘
 * @endverbatim
 * 
 * @section features Key Features
 * 
 * - **Real-Time Safe**: Lock-free buffers, no dynamic allocations in control loop
 * - **Smooth Motion**: Minimum Jerk Trajectory (C² continuous)
 * - **Safety Filtering**: Enforces velocity, acceleration, jerk limits
 * - **Generalizable**: Works with any number of joints (DOF)
 * - **Robot-Agnostic**: Integrates with any robot via ROS 2 hardware interface
 * 
 * @section lifecycle Controller Lifecycle
 * 
 * The controller follows the ROS 2 lifecycle pattern:
 * 
 * @code
 * on_init()      - Load parameters, validate configuration
 *   ↓
 * on_configure() - Allocate buffers, initialize hardware interfaces
 *   ↓
 * on_activate()  - Start trajectory generation, enable control loop
 *   ↓
 * [update() called 1000 times per second]
 *   ↓
 * on_deactivate() - Stop control, reset state
 *   ↓
 * on_cleanup()   - Release resources
 * @endcode
 * 
 * @section parameters Configuration Parameters
 * 
 * YAML configuration example:
 * @code
 * ai_control_bridge_controller:
 *   ros__parameters:
 *     num_joints: 7                    # Number of joints
 *     joint_names:                     # ROS 2 control names
 *       - joint1
 *       - joint2
 *       # ... up to 7 joints
 *     control_frequency: 1000.0        # Hz (must match hardware)
 *     trajectory_duration: 1.0         # seconds for quintic
 *     command_interface: "position"    # "position" or "velocity"
 * @endcode
 * 
 * @section update_rate High-Frequency Update Loop (1 kHz)
 * 
 * The update() method runs **1000 times per second** in a realtime thread.
 * It must complete in < 1 ms (1000 µs) consistently:
 * 
 * @code
 * Time Budget (1 ms):
 * ├─ Read joint states: 10-20 µs
 * ├─ Check for new waypoint: 50-100 ns
 * ├─ Generate trajectory point: 20-50 µs
 * ├─ Apply safety constraints: 5-10 µs
 * ├─ Send command to hardware: 100-500 µs
 * └─ Total: 150-600 µs (< 1000 µs ✓)
 * 
 * Headroom for kernel scheduling, cache misses, etc: 400-850 µs
 * @endcode
 * 
 * @section generality Generalized for Any Robot
 * 
 * This implementation is NOT specific to BCR Arm:
 * 
 * - ✓ Accepts any number of joints via parameter
 * - ✓ Works with any control interface (position/velocity)
 * - ✓ Integrates via standard ROS 2 hardware interface
 * - ✓ Safety limits per-joint (customizable)
 * 
 * **Example robots:**
 * - 6-DOF industrial manipulators
 * - 7-DOF collaborative robots
 * - Humanoid legs (12+ DOF)
 * - Mobile manipulators (3 + 7 DOF)
 * 
 * @section thread_safety Thread Safety
 * 
 * - **Control thread (realtime)**: Runs update(), must not block
 * - **Inference thread (non-realtime)**: Publishes waypoints
 * - **Communication**: Lock-free ring buffer (RealtimeSafeBuffer)
 * 
 * No mutexes, no condition variables, no dynamic allocations
 * in the realtime control path.
 * 
 * @section debugging Debugging and Monitoring
 * 
 * Monitor controller performance:
 * @code
 * # Check controller state
 * ros2 controller list
 * ros2 controller state ai_control_bridge
 * 
 * # Monitor waypoints and commands
 * ros2 topic echo /ai_waypoints
 * ros2 topic hz /ai_waypoints      # Should be ~50 Hz
 * ros2 topic hz /joint_states      # Should be ~1000 Hz
 * 
 * # Check for constraint violations in logs
 * ros2 run rqt_console rqt_console
 * @endcode
 * 
 * @section extending Extending for Your Robot
 * 
 * To adapt this controller for a new robot:
 * 
 * 1. **Create hardware interface**
 *    @code
 *    #include <hardware_interface/system_interface.hpp>
 *    class MyRobotHW : public hardware_interface::SystemInterface { ... };
 *    @endcode
 * 
 * 2. **Create URDF model**
 *    ```xml
 *    <robot name="my_robot">
 *      <joint name="joint1" type="revolute"> ... </joint>
 *      <!-- Define all joints -->
 *    </robot>
 *    ```
 * 
 * 3. **Create ros2_control description**
 *    ```yaml
 *    hardware:
 *      type: my_namespace/MyRobotHW
 *    ```
 * 
 * 4. **Create launch file** (see launch/ folder for examples)
 * 
 * 5. **Configure safety limits** for your specific robot
 * 
 * @author GSoC Contributors
 * @date 2026
 * @see RealtimeSafeBuffer, MinimumJerkTrajectory, SafetyFilter
 */
class AIControllerBridge : public controller_interface::ControllerInterface {
public:
    AIControllerBridge();
    virtual ~AIControllerBridge() = default;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
     * @brief Main control loop update function (called at high frequency).
     * Real-time safe - no allocations, no locks, deterministic.
     */
    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    // Parameters
    size_t num_joints_;
    double control_frequency_;  // Hz
    double interpolation_dt_;
    std::string command_interface_;  // "position" or "velocity"
    
    // Joint names for identification and debugging
    std::vector<std::string> joint_names_;

    // Current joint state (updated from hardware) - FIXED SIZE for real-time safety
    std::array<double, 16> current_position_{};      // Up to 16 DOF
    std::array<double, 16> current_velocity_{};
    std::array<double, 16> current_effort_{};

    // Pending AI target (set from non-RT subscriber callback) - FIXED SIZE
    std::array<double, 16> pending_target_{};
    std::atomic<bool> has_pending_target_{false};

    // Per-joint trajectory generation (MJT) - FIXED SIZE
    std::array<MinimumJerkTrajectory::Coefficients, 16> per_joint_trajectory_{};
    int64_t trajectory_start_time_{0};  // nanoseconds
    std::atomic<bool> trajectory_active_{false};

    // Realtime-safe buffers
    std::unique_ptr<RealtimeSafeBuffer<std::array<double, 16>, 200>> waypoint_buffer_;

    // Safety filter
    SafetyFilter safety_filter_;

    // Metrics - thread-safe for access from multiple threads
    struct Metrics {
        std::atomic<uint64_t> update_count{0};
        std::atomic<double> max_latency{0.0};
        std::atomic<int> constraint_violations{0};
    } metrics_;

    // Topic publishers/subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr waypoints_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_publisher_;

    // Callbacks
    void waypoints_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace ai_control_bridge_controller
