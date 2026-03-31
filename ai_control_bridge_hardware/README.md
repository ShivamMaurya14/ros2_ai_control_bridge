# ai_control_bridge_hardware

Hardware interface implementations and robot descriptions for AI Control Bridge. **Reference implementation for connecting robots.**

## Overview

This package provides reference implementations for connecting robots to the AI Control Bridge framework:

1. **Hardware Interface** - BCR ARM example showing how to connect any robot
2. **URDF Description** - Robot kinematic structure definition
3. **Integration Template** - Starting point for adapting to other robots

**Key Principle:** Use the BCR ARM example as a template to adapt the AI Control Bridge to your specific robot.

## Package Structure

```
ai_control_bridge_hardware/
├── CMakeLists.txt                          # Build configuration
├── package.xml                             # Package metadata
├── README.md                               # This file
├── include/
│   └── ai_control_bridge_hardware/
│       └── bcr_arm_hardware_interface.hpp # BCR ARM hardware interface (template)
├── src/
│   └── bcr_arm_hardware_interface.cpp     # BCR ARM implementation (template)
└── urdf/
    └── (URDF files for robot descriptions)
```



## Getting Started: Adapt for Your Robot

The BCR ARM example files serve as a template for connecting other robots. To adapt for your robot:

### Step 1: Study BCR ARM Implementation

The existing implementation shows the required interface:

```cpp
// include/ai_control_bridge_hardware/bcr_arm_hardware_interface.hpp
// Shows how to implement hardware interface for a robot
```

**Key components to implement:**
- Constructor to initialize robot connection
- Read method to get current joint states
- Write method to send joint commands
- State management for hardware communication

### Step 2: Create Your Hardware Interface

Create a new header and implementation file for your robot:

```bash
# Copy the BCR template
cp include/ai_control_bridge_hardware/bcr_arm_hardware_interface.hpp \
   include/ai_control_bridge_hardware/your_robot_hardware_interface.hpp

cp src/bcr_arm_hardware_interface.cpp \
   src/your_robot_hardware_interface.cpp
```

Modify `your_robot_hardware_interface.cpp` to:
- Connect to your robot's native APIs
- Implement joint state reading via your robot's drivers
- Implement joint command writing via your robot's controllers
- Handle your robot's specific communication protocol

### Step 3: Define URDF Description

Create or update robot URDF file in the `urdf/` directory to describe:
- Kinematic chain (links and joints)
- Joint limits (min/max position, velocity, effort)
- Physical properties (mass, inertia)
- Visual and collision geometry

### Step 4: Configure CMakeLists.txt

Update `CMakeLists.txt` to:
- Add your robot-specific dependencies
- Register your new hardware interface as a plugin
- Link against your robot's communication libraries

## Features

| Interface | Type | Purpose |
|-----------|------|---------|
| `position` | Command | Set target joint position |
| `velocity` | Command | Set target joint velocity |
| `effort` | Command | Set target joint torque/force |
| `position` | State | Read actual joint position |
| `velocity` | State | Read actual joint velocity |
| `effort` | State | Read actual joint torque/force |

## Example: Implementing Hardware Interface

The BCR ARM implementation demonstrates the required interface pattern:

```cpp
#include "hardware_interface/hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ai_control_bridge_hardware {

class BCRARMHardware : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;
    
    CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    
    CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    
    CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;
    
    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;
    
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;
};

}  // namespace ai_control_bridge_hardware
```

## Building

```bash
cd /path/to/workspace
colcon build --packages-select ai_control_bridge_hardware
source install/setup.bash
```

## Integration with AI Control Bridge

This hardware interface integrates with:
- **ai_control_bridge_controller** - Real-time control at 1000 Hz
- **ai_inference_processor** - Policy inference at ~50 Hz
- **ros2_control** - Standard ROS 2 hardware abstraction

See the individual package READMEs for integration details.

## URDF Structure

### Example Joint Definition

```xml
<joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" 
            effort="150" velocity="1.57"/>
    <dynamics damping="0.1" friction="0.0"/>
    
    <!-- ros2_control interface -->
    <ros2_control name="joint1">
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <command_interface name="position"/>
    </ros2_control>
</joint>
```

### Example Gazebo Plugin

```xml
<gazebo reference="joint1">
    <implicitSpringDamper>true</implicitSpringDamper>
    <kp>1000</kp>
    <kd>100</kd>
</gazebo>
```

## Usage

### 1. Load URDF

```bash
# Display robot model
ros2 run rviz2 rviz2
# Then: Add → RobotModel → Topic: /robot_description

# Or programmatically:
robot_description = xacro.process_file('urdf/bcr_arm.urdf.xacro').toxml()
```

### 2. Spawn in Gazebo

```bash
# Terminal 1: Start Gazebo
gazebo --physics ode

# Terminal 2: Spawn robot
ros2 launch ai_control_bridge_hardware robot_bringup.launch.py \
    sim_mode:=gazebo
```

### 3. Visualize in RViz

```bash
# Terminal 3: Start RViz
ros2 launch ai_control_bridge_hardware rviz.launch.py
```

## Configuration

### Create `config/robot_config.yaml`

```yaml
robot_hardware:
  ros__parameters:
    # Hardware interface type
    hardware_interface: "gazebo"  # or "real"
    
    # Robot description
    robot_description_file: "urdf/bcr_arm.urdf.xacro"
    
    # Joint configuration
    joints:
      - name: joint1
        type: revolute
        control_interface: position
        
      - name: joint2
        type: revolute
        control_interface: position
    
    # Communication
    update_frequency: 1000  # Hz
    
    # Simulation (if applicable)
    simulation:
      sim_granularity: 0.001  # seconds per step
      real_time_factor: 1.0
```

### Create Launch File

Create `launch/robot_bringup.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='gazebo',
            description='Simulation mode: gazebo or real'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': '$(command xacro $(find ai_control_bridge_hardware)/urdf/bcr_arm.urdf.xacro)'}],
        ),
        
        Node(
            package='ai_control_bridge_hardware',
            executable='gazebo_hardware_node',
            condition=IfCondition(PythonExpression(["'", sim_mode, "' == 'gazebo'"]))
        ),
    ])
```

## Commissioning Real Hardware

### Step 1: Identify Hardware

```bash
# List connected devices
ls /dev/tty*

# Find robot control interface
rostopic list | grep control
```

### Step 2: Create Hardware Interface

Implement `SystemInterface` for your specific robot:

```cpp
class RealRobotHardware : public hardware_interface::SystemInterface {
    // Read from real hardware
    // Write to real hardware
};
```

### Step 3: Configure Parameters

```yaml
robot_hardware:
  ros__parameters:
    hardware_interface: "real"
    device_port: "/dev/ttyUSB0"
    baud_rate: 115200
    update_frequency: 1000
```

### Step 4: Test Communication

```bash
# Verify hardware is reachable
ros2 service call /hardware/check_connection std_srvs/Empty

# Read current state
rostopic echo /robot_state
```

## Troubleshooting

### URDF Won't Load

```bash
# Check URDF syntax with xacro
xacro urdf/bcr_arm.urdf.xacro | xmllint --format -

# Verify in RViz
ros2 run rviz2 rviz2
# Add RobotModel and check /robot_description topic
```

### Hardware Not Detected

```bash
# List available hardware interfaces
ros2 control list_hardwares

# Check ros2_control configuration
grep -r "hardware_interface" urdf/

# View control node logs
ros2 node info /controller_manager
```

### Gazebo Physics Unstable

```yaml
# Adjust physics parameters in gazebo.xacro
<physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.81</gravity>
</physics>
```

## Integration with AI Control Bridge

### Full System Architecture

```
┌─────────────────────────────────────────────────┐
│ AI Control Bridge System                        │
├─────────────────────────────────────────────────┤
│ PreprocessorCore        [ai_control_bridge_core]│
│   ↓ (observations)                              │
│ InferenceCore           [ai_inference_processor]│
│   ↓ (waypoints)                                 │
│ PostprocessorCore       [ai_control_bridge_...]│
│   ↓ (commands)                                  │
│ Hardware Interface      [ai_control_bridge_...] │
│   ↓ (joint commands)                            │
│ Robot / Gazebo          [ai_control_bridge_...] │
└─────────────────────────────────────────────────┘
```

### Launch Full System

```bash
# Terminal 1: Start robot (real or Gazebo)
ros2 launch ai_control_bridge_hardware robot_bringup.launch.py

# Terminal 2: Start AI pipeline
ros2 launch ai_control_bridge inference_processor.launch.py

# Terminal 3: Initialize controllers
ros2 control load_controller ai_control_bridge_controller

# Terminal 4: Activate controller
ros2 control set_controller_state ai_control_bridge_controller active

# Terminal 5: Monitor
ros2 topic echo /joint_states
```

## Advanced Topics

### Custom Dynamics

Implement physics simulation beyond default Gazebo:

```cpp
class CustomDynamicsHardware : public SystemInterface {
private:
    Eigen::VectorXd gravity_torque;
    Eigen::MatrixXd inertia_matrix;
    
    void compute_gravity_compensation() {
        // Compute gravity torque based on configuration
    }
};
```

### Sensor Integration

Add additional sensors to URDF:

```xml
<link name="force_torque_sensor">
    <sensor name="ft_sensor" type="force_torque">
        <force_torque>
            <frame>child</frame>
            <measure_direction>child_to_parent</measure_direction>
        </force_torque>
    </sensor>
</link>
```

### Multi-Robot Support

Control multiple robots in same system:

```python
# launch/multi_robot_bringup.launch.py
robot1 = Node(package='ai_control_bridge_hardware', 
              executable='hardware_node',
              namespace='robot1')
robot2 = Node(package='ai_control_bridge_hardware',
              executable='hardware_node',
              namespace='robot2')
```

## References

- [URDF Specification](http://wiki.ros.org/urdf)
- [ros2_control Documentation](https://control.ros.org/)
- [Gazebo Documentation](http://gazebosim.org/)
- [ROS 2 Hardware Interface](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Hardware-Interfaces.html)

## License

Apache License 2.0 - See LICENSE in root directory

## Support

For issues:
- **Core pipeline**: See `ai_control_bridge_core/README.md`
- **Controller**: See `ai_control_bridge_controller/README.md`
- **Inference**: See `ai_inference_processor/README.md`
