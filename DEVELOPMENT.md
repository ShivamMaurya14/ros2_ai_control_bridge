# Development Guide - AI Control Bridge

This guide helps developers understand the architecture and extend the AI Control Bridge for their own robots.

## Architecture Overview

The AI Control Bridge is built on three main stages:

### Stage 1: PreprocessorCore - Observation Collection

**Purpose**: Gather sensor data from the robot and prepare it for inference

**Responsibilities**:
- Read joint states, IMU data, force/torque sensors
- Normalize sensor values to expected input ranges
- Manage observation history if needed
- Run at high frequency (1 kHz typical)

**Example Implementation**:
```cpp
class MyRobotPreprocessor : public PreprocessorCore {
public:
    void collect_observations(std::vector<double>& obs) override {
        obs.resize(12);
        
        // Read joint states via ros2_control interface
        obs[0] = joint_positions_[0];  // Joint 1 position
        obs[1] = joint_positions_[1];  // Joint 2 position
        // ... more joints
        
        // Read joint velocities
        obs[6] = joint_velocities_[0];  // Joint 1 velocity
        // ... more velocities
        
        // Normalize to [-1, 1] range expected by model
        for (auto& val : obs) {
            val = (val - mean_) / std_dev_;
        }
    }
};
```

### Stage 2: InferenceCore - Policy Inference

**Purpose**: Run neural network policy to generate actions/waypoints

**Responsibilities**:
- Load pre-trained model (ONNX, TensorRT, PyTorch, etc)
- Run inference on preprocessed observations
- Handle model failures gracefully
- Run at low frequency (~50 Hz, non-realtime)

**Example Implementation with ONNX**:
```cpp
class ONNXInferenceProcessor : public InferenceCore {
private:
    std::unique_ptr<Ort::Session> session_;
    
public:
    ONNXInferenceProcessor(const std::string& model_path) {
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "inference");
        Ort::SessionOptions session_options;
        session_options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(
            env, model_path.c_str(), session_options);
    }
    
    bool run_inference(const std::vector<double>& obs,
                       std::vector<double>& actions) override {
        try {
            // Prepare input tensor
            auto input_tensor = Ort::Value::CreateTensor<float>(
                memory_info, obs.data(), obs.size(), 
                input_shape.data(), input_shape.size());
            
            // Run inference
            const char* input_names[] = {"observations"};
            const char* output_names[] = {"actions"};
            auto output_tensors = session_->Run(
                Ort::RunOptions{nullptr},
                input_names, &input_tensor, 1,
                output_names, 1);
            
            // Extract results
            float* output_data = output_tensors[0].GetTensorMutableData<float>();
            actions.assign(output_data, output_data + num_actions_);
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Inference error: %s", e.what());
            return false;
        }
    }
};
```

### Stage 3: PostprocessorCore - Action Processing & Trajectory Generation

**Purpose**: Convert low-frequency waypoints to high-frequency smooth commands

**Responsibilities**:
- Generate smooth trajectories (quintic, cubic spline, etc)
- Enforce safety constraints (position, velocity limits)
- Command robot at high frequency (1 kHz)
- Must be realtime-safe (no locks, bounded time)

**Example Implementation**:
```cpp
class TrajectoryPostprocessor : public PostprocessorCore {
private:
    MinimumJerkTrajectory trajectory_;
    SafetyFilter safety_filter_;
    
public:
    const std::vector<double>& process(
        const std::vector<double>& waypoints) override {
        
        // Generate trajectory from current position to waypoint
        trajectory_.compute(
            current_position_,
            waypoints,
            trajectory_duration_);  // e.g., 1.0 second
        
        // Evaluate trajectory at current time
        trajectory_.evaluate(
            current_time_,
            commands_.position,
            commands_.velocity,
            commands_.acceleration);
        
        // Apply safety constraints
        safety_filter_.enforce_limits(commands_);
        
        return commands_.position;
    }
};
```

## Extending for Your Robot

### Step 1: Create Hardware Interface

```cpp
// my_robot_hardware.hpp
#include <hardware_interface/hardware_interface.hpp>

class MyRobotHardwareInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& hardware_info) override {
        // Initialize connection to your robot
        robot_->connect("192.168.1.100", 10000);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override {
        // Read current state from robot
        joint_states_ = robot_->read_state();
        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override {
        // Send commands to robot
        robot_->write_commands(command_interfaces_);
        return hardware_interface::return_type::OK;
    }
};
```

### Step 2: Create ROS 2 Control Configuration

```yaml
# my_robot_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    use_sim_time: true
    
    controllers:
      joint_state_broadcaster:
        module: controller_manager::ros2_control::JointStateBroadcaster
        
      ai_control_bridge:
        module: ai_control_bridge_controller::AIControllerBridge
        params_file: ai_bridge_params.yaml
```

### Step 3: Create Launch File

```python
# my_robot_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch ros2_control node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                'my_robot_hardware.yaml',
                'my_robot_controllers.yaml',
            ],
        ),
        # Launch state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        ),
        # Launch AI bridge controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ai_control_bridge', '-c', '/controller_manager'],
        ),
        # Launch inference processor
        Node(
            package='ai_inference_processor',
            executable='ai_inference_processor_node',
            parameters=['ai_inference_config.yaml'],
        ),
    ])
```

## Real-Time Safety

The PostprocessorCore runs in the realtime control loop. Therefore:

### ✅ DO:
```cpp
// Use pre-allocated buffers
std::vector<double> my_buffer;  // Pre-sized in constructor

// Use bounded algorithms
for (size_t i = 0; i < num_joints_; ++i) {
    position_[i] = compute_trajectory_point(i, time);
}

// Use atomic operations for thread-safe communication
#include <atomic>
std::atomic<bool> new_waypoint_available{false};
```

### ❌ DON'T:
```cpp
// Dynamic allocations in control loop
auto buffer = std::make_unique<std::vector<double>>(100);  // NO!

// Blocking operations
std::lock_guard<std::mutex> lock(mutex_);  // NO!
std::condition_variable::wait();            // NO!

// Slow algorithms
std::sort(data.begin(), data.end());  // NO!

// System calls
std::cout << "Debug output";  // NO!
```

## Testing Your Extension

### Unit Tests
```cpp
TEST(MyRobotHardware, ReadsJointStates) {
    MyRobotHardwareInterface hw;
    hw.on_init(hardware_info);
    
    auto status = hw.read(now, period);
    EXPECT_EQ(status, hardware_interface::return_type::OK);
    EXPECT_EQ(joint_positions_.size(), 6);
}
```

### Integration Tests
```bash
# Test with real robot
ros2 launch my_robot_bringup.launch.py
ros2 launch ai_control_bridge_core ai_bridge_test.launch.py

# Monitor topics
ros2 topic echo /joint_states
ros2 topic echo /waypoints
```

### Performance Profiling
```bash
# Run with profiling
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch my_robot_bringup.launch.py
# Use perf/valgrind to profile
```

## Common Pitfalls & Solutions

### Issue: "Realtime kernel is not running"
**Impact**: Occasional jitter in control loop
**Solution**: Use `preempt-rt` kernel patch or PREEMPT_VOLUNTARY

### Issue: "Inference is too slow"
**Impact**: Waypoints arrive too slowly, robot has nothing to do
**Solution**:
- Use TensorRT for GPU acceleration
- Quantize model (FP32 → INT8)
- Reduce observation history length
- Profile with `nsys` or `perf`

### Issue: "Trajectory is jerky"
**Impact**: Uncomfortable/unsafe motion
**Solution**:
- Increase trajectory duration (slower transitions)
- Use lower control frequency (if possible)
- Add acceleration/jerk limits in PostprocessorCore

### Issue: "Robot doesn't respond to waypoints"
**Impact**: Commanded positions not changing
**Solution**:
1. Check topic names match (observations → waypoints)
2. Verify controller is ACTIVE (`ros2 controller list`)
3. Check safety limits aren't violated
4. Verify hardware interface is reading/writing correctly

## Debugging

### Use ROS 2 tools:
```bash
# Monitor all topics
ros2 topic list
ros2 topic hz /waypoints
ros2 topic echo /joint_states

# Check controller state
ros2 controller list
ros2 controller state ai_control_bridge

# Monitor logs
ros2 run rqt_console rqt_console  # GUI logging

# Record data for analysis
ros2 bag record /joint_states /waypoints /ai_commands
```

### Use C++ debugging:
```cpp
// Add diagnostic measurements
auto start = std::chrono::high_resolution_clock::now();
// ... do work ...
auto elapsed = std::chrono::high_resolution_clock::now() - start;
RCLCPP_INFO(get_logger(), "Update took %.3f ms",
           std::chrono::duration<double, std::milli>(elapsed).count());
```

## References

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ONNX Runtime](https://onnxruntime.ai/docs/)
- [Real-Time Programming Guide](https://man7.org/linux/man-pages/man7/rt_sigaction.7.html)

---

Happy developing! 🚀
