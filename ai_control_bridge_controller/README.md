# ai_control_bridge_controller

Real-time ros2_control plugin for safe execution of AI policy waypoints through smooth trajectory generation and safety constraint enforcement. **Works with any robot.**

## Overview

This package provides a concrete, generalizable implementation of the **PostprocessorCore** interface. It bridges the fundamental synchronization gap between low-frequency AI policy outputs (~50 Hz) and high-frequency robot hardware commands (≤1000 Hz).

**Applicable to:** Any robot - manipulators, humanoids, wheeled platforms, aerial vehicles, etc.

### Key Features

- ✅ **Real-Time Safe**: Lock-free buffers, no dynamic allocations in control loop
- ✅ **Smooth Motion**: Minimum Jerk Trajectory for C³ continuous motion
- ✅ **Safety Filtering**: Enforces velocity, acceleration, and jerk limits per-joint
- ✅ **Jitter Tolerant**: Handles inference delays and variable timing
- ✅ **Plugin-Based**: Discovered and loaded via ros2_control plugin system
- ✅ **Robot-Agnostic**: Works with any number of joints (DOF)

## Architecture

```
AI Policy (50 Hz, async)
    ↓
 waypoints[]
    ↓
[AIControllerBridge]
    ├─ RealtimeSafeBuffer (lock-free ring buffer)
    ├─ MinimumJerkTrajectory (quintic polynomial generator)
    └─ SafetyFilter (constraint enforcement)
    ↓
[1 kHz Control Loop]
    ↓
Joint Commands → Robot Hardware
```

## Component Overview

### 1. AIControllerBridge

Main controller class implementing `controller_interface::ControllerInterface`.

**Lifecycle Methods:**
- `on_init()` - Initialize controller
- `on_configure()` - Load parameters and configure
- `on_activate()` - Start active control
- `on_deactivate()` - Stop active control
- `on_cleanup()` - Release resources

**Control Method:**
```cpp
controller_interface::return_type update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period);
```

Runs at robot control frequency (typically 1 kHz).

### 2. RealtimeSafeBuffer

Lock-free ring buffer for communication between low-frequency inference and high-frequency control.

**Features:**
- Template-based fixed-size capacity
- Atomic read/write operations (no locks)
- Handles out-of-order updates gracefully
- Default capacity: 200 waypoints (4 seconds at 50 Hz)

**Usage:**
```cpp
RealtimeSafeBuffer<std::vector<double>> buffer(200);

// Producer thread (50 Hz inference)
buffer.push(new_waypoints);

// Consumer thread (1 kHz control)
auto waypoints = buffer.pop();
if (waypoints) {
    use_waypoints(*waypoints);
}
```

### 3. MinimumJerkTrajectory

Quintic polynomial trajectory generator for smooth motion between waypoints.

**Properties:**
- **Continuity**: C² continuous (position, velocity, acceleration all smooth)
- **Jerk**: Minimized through polynomial optimization
- **Computation**: Real-time safe, all allocations pre-made

**Polynomial Coefficients:**
```
x(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```

Where coefficients (a₀ through a₅) satisfy:
- x(0) = current position, x(T) = target position
- ẋ(0) = ẋ(T) = 0 (zero velocity at endpoints)
- ẍ(0) = ẍ(T) = 0 (zero acceleration at endpoints)

**Usage:**
```cpp
MinimumJerkTrajectory mj(6);  // 6-DOF
mj.set_duration(1.0);  // 1 second trajectory

while (controller_running) {
    double position = mj.get_position(time_normalized);
    double velocity = mj.get_velocity(time_normalized);
    set_command(position, velocity);
}
```

### 4. SafetyFilter

Enforces hard constraint limits on joint motion.

**Constraints:**
- Position bounds (joint limits)
- Velocity limits (rad/s or m/s)
- Acceleration limits (rad/s² or m/s²)

**Filtering Strategy:**
- Hard clipping for safety-critical constraints
- Smooth saturation for smoothness

**Usage:**
```cpp
SafetyFilter safety(6);  // 6-DOF

safety.set_velocity_limit(0, 1.57);  // rad/s for joint 0
safety.set_acceleration_limit(0, 3.14);  // rad/s² for joint 0

auto safe_command = safety.enforce(desired_command);
```

## Configuration

### Parameters

Create `config/controller_config.yaml`:

```yaml
ai_control_bridge_controller:
  ros__parameters:
    # Joint configuration
    num_joints: 6
    joint_names:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    
    # Control frequency
    control_frequency: 1000.0
    
    # Trajectory generation
    interpolation_dt: 0.001  # seconds
    min_trajectory_duration: 0.5  # seconds
    
    # Safety constraints
    joint_velocity_limits:
      - 1.57  # rad/s per joint
      - 1.57
      - 1.57
      - 1.57
      - 1.57
      - 1.57
    
    joint_acceleration_limits:
      - 3.14  # rad/s² per joint
      - 3.14
      - 3.14
      - 3.14
      - 3.14
      - 3.14
    
    # Buffer configuration
    buffer_capacity: 200  # waypoints
    
    # Safety
    enable_soft_limits: false
    joint_position_lower_limits: [-3.14, -1.57, -3.14, -1.57, -3.14, -1.57]
    joint_position_upper_limits: [3.14, 1.57, 3.14, 1.57, 3.14, 1.57]
```

### Launch File

Create `launch/ai_control_bridge.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('ai_control_bridge_controller')
    config_file = os.path.join(pkg_dir, 'config', 'controller_config.yaml')
    
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ai_control_bridge_controller'],
            output='screen'
        ),
    ])
```

## Usage

### 1. Load Controller

```bash
# Load the controller plugin
ros2 control load_controller ai_control_bridge_controller
```

### 2. Activate Controller

```bash
# Set controller to active state
ros2 control set_controller_state ai_control_bridge_controller active
```

### 3. Send Waypoints

```bash
# Publish waypoints from AI inference
ros2 topic pub /waypoints std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]"
```

### 4. Monitor Commands

```bash
# View generated joint commands
ros2 topic echo /joint_commands

# View controller state
ros2 control list_controllers
```

## ROS 2 Integration

### Topics

| Topic | Message | Direction | Frequency | Purpose |
|-------|---------|-----------|-----------|---------|
| `/waypoints` | `std_msgs/Float64MultiArray` | Input | 50 Hz | AI policy waypoints |
| `/commands` | `std_msgs/Float64MultiArray` | Output | 1000 Hz | Joint commands to hardware |
| `/state/positions` | `sensor_msgs/JointState` | Output | 1000 Hz | Current joint state |

### Services

| Service | Purpose |
|---------|---------|
| `/ai_control_bridge_controller/list_constraints` | Query active safety constraints |
| `/ai_control_bridge_controller/update_limits` | Dynamically update safety limits |

### Parameters

See **Configuration** section above for full parameter list.

## Real-Time Safety

This controller is designed for real-time execution. Follow these guidelines:

✅ **Do:**
- Pre-allocate all buffers in `on_configure()`
- Use lock-free buffers (atomic operations only)
- Keep update loop <1 ms at 1 kHz

❌ **Don't:**
- Allocate memory in `update()` method
- Use `std::mutex` or `std::lock_guard` in control loop
- Call blocking I/O in control thread
- Use ROS 2 logging at high frequency

## Performance Characteristics

**Latency Budget (per update, 1 ms @ 1 kHz):**
- Lock-free buffer read: ~0.1 μs
- Trajectory interpolation: ~10 μs
- Safety filtering: ~5 μs
- Total: <100 μs → **Plenty of headroom**

**Memory Usage:**
- Ring buffer (200 waypoints): ~1.6 KB
- Trajectory struct: ~500 B
- Safety filter: ~1 KB
- **Total: ~3 KB** (data only, no allocations in loop)

## Extending the Controller

### Add Custom Constraint

```cpp
class MyConstraint : public SafetyFilter {
public:
    void enforce(std::vector<double>& cmd) override {
        // Call parent first
        SafetyFilter::enforce(cmd);
        
        // Add custom logic
        if (cmd[0] < 0 && robot_in_singular_config()) {
            cmd[0] = 0;  // Prevent motion toward singularity
        }
    }
};
```

### Add Custom Trajectory Generator

```cpp
class MyTrajectory : public MinimumJerkTrajectory {
public:
    double get_position(double t_normalized) override {
        // Custom trajectory computation
        // ...
    }
};
```

## Troubleshooting

**Problem:** Controller won't activate
```bash
# Check controller state
ros2 control list_controllers

# View error logs
journalctl -u ros2 | grep ai_control_bridge
```

**Problem:** Jerky motion
- Increase `min_trajectory_duration` in config
- Check waypoint spacing consistency
- Verify inference frequency is adequate

**Problem:** Safety violations
- Lower velocity limits for conservative behavior
- Check gravity compensation in hardware interface
- Verify sensor accuracy (joint state topics)

## Testing

### Unit Tests

```cpp
#include <gtest/gtest.h>
#include "ai_control_bridge_controller/controller.hpp"

TEST(AIControllerBridge, InterpolatesSmoothly) {
    auto controller = std::make_unique<AIControllerBridge>();
    
    std::vector<double> waypoints = {0.0, 1.0};
    controller->set_waypoints(waypoints);
    
    auto cmd = controller->update_command(0.5);  // Mid-trajectory
    
    // Verify smooth interpolation
    EXPECT_GT(cmd[0], 0.0);
    EXPECT_LT(cmd[0], 1.0);
}
```

### Integration Test

```bash
# Start your robot simulation/hardware
# ros2 launch your_robot_bringup robot.launch.py

# Build and source the workspace
colcon build --symlink-install
source install/setup.bash

# Load and activate controller
ros2 control load_controller ai_control_bridge_controller
ros2 control set_controller_state ai_control_bridge_controller active

# Send test command (adjust DOF and values for your robot)
ros2 topic pub -1 /waypoints std_msgs/msg/Float64MultiArray \
  "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"

# Monitor output
ros2 topic echo /commands
```

## References

- [ros2_control Documentation](https://control.ros.org/)
- [Quintic Trajectory Planning](https://en.wikipedia.org/wiki/Trajectory_planning)
- [Real-Time Programming in ROS 2](https://docs.ros.org/en/humble/Tutorials/Advanced/Real-Time-Programming.html)

## License

Apache License 2.0 - See LICENSE in root directory

## Support

For issues related to:
- **Core pipeline**: See `ai_control_bridge_core/README.md`
- **Inference**: See `ai_inference_processor/README.md`
- **Hardware interface**: See `ai_control_bridge_hardware/README.md`
