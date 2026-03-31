# API Reference - AI Control Bridge Core

Complete documentation of the core abstract interfaces and how to extend the framework.

## Core Interfaces

### PreprocessorCore

**Purpose**: Gather and prepare sensor observations for policy inference.

**Inheritance**: Inherits from `ai_control_bridge::SensorInterface`

**Key Methods**:

```cpp
/// @brief Collect current sensor observations from the robot
/// @param[out] observations Vector of normalized sensor values
/// @details Called at high frequency (typically 1 kHz) by the control loop.
///          Must be real-time safe - no allocations, system calls, or locks.
/// @note Observations should be normalized to [-1, 1] or [0, 1] range
///       matching the neural network training data distribution
virtual void collect_observations(std::vector<double>& observations) = 0;

/// @brief Get the number of observation values this preprocessor outputs
/// @return Dimension of observation vector
virtual size_t get_observation_dim() const = 0;

/// @brief Get human-readable names of observation components
/// @return Vector of observation names (e.g., "joint_1_position", "joint_1_velocity")
virtual std::vector<std::string> get_observation_names() const = 0;
```

**Usage in Plugin**:

```cpp
class MyRobotPreprocessor : public PreprocessorCore {
private:
    size_t num_joints_ = 6;
    std::array<double, 12> observations_;  // Pre-allocated!
    
public:
    void collect_observations(std::vector<double>& obs) override {
        // Fast path: reuse pre-allocated buffer
        obs.resize(12);  // Assuming 6 DOF: 6 positions + 6 velocities
        
        // Copy from cached observations (updated by hardware interface)
        std::copy(observations_.begin(), observations_.end(), obs.begin());
        
        // Normalize to [-1, 1]
        for (auto& val : obs) {
            val = (val - mean_) / std_dev_;
        }
    }
    
    size_t get_observation_dim() const override {
        return 12;
    }
    
    std::vector<std::string> get_observation_names() const override {
        return {
            "joint_1_position", "joint_2_position", "joint_3_position",
            "joint_4_position", "joint_5_position", "joint_6_position",
            "joint_1_velocity", "joint_2_velocity", "joint_3_velocity",
            "joint_4_velocity", "joint_5_velocity", "joint_6_velocity"
        };
    }
};
```

### InferenceCore

**Purpose**: Run pre-trained policy on observations to generate desired waypoints.

**Key Methods**:

```cpp
/// @brief Run policy inference on observations
/// @param[in] observations Preprocessed sensor observations ([-1, 1] range)
/// @param[out] waypoints Target waypoints from policy (typically [-1, 1])
/// @return true if inference successful, false if failed
/// @details Called asynchronously at ~50 Hz by inference thread.
///          Can use locks, dynamic allocations (inference is not real-time critical).
/// @throws std::runtime_error if model not loaded or inference fails
virtual bool infer(const std::vector<double>& observations,
                   std::vector<double>& waypoints) = 0;

/// @brief Get the action dimension this inference processor outputs
/// @return Number of action components (usually equals num joints)
virtual size_t get_action_dim() const = 0;

/// @brief Load policy model from file
/// @param[in] model_path Path to model file (format depends on implementation)
/// @return true if loaded successfully
/// @throws std::runtime_error if file not found or model invalid
virtual bool load_model(const std::string& model_path) = 0;

/// @brief Check if model is successfully loaded
/// @return true if model ready for inference
virtual bool is_ready() const = 0;
```

**Usage with ONNX Runtime**:

```cpp
class ONNXInferenceProcessor : public InferenceCore {
private:
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    
public:
    bool load_model(const std::string& model_path) override {
        try {
            Ort::SessionOptions options;
            options.SetGraphOptimizationLevel(
                GraphOptimizationLevel::ORT_ENABLE_ALL);
            session_ = std::make_unique<Ort::Session>(
                env_, model_path.c_str(), options);
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load model: %s", e.what());
            return false;
        }
    }
    
    bool infer(const std::vector<double>& observations,
               std::vector<double>& waypoints) override {
        if (!session_) return false;
        
        try {
            // Convert double to float for ONNX
            std::vector<float> obs_float(observations.begin(),
                                        observations.end());
            
            // Prepare input tensor
            auto memory_info = Ort::MemoryInfo::CreateCpu(
                OrtArenaAllocator, OrtMemTypeDefault);
            auto input_tensor = Ort::Value::CreateTensor<float>(
                memory_info, obs_float.data(), obs_float.size(),
                input_shape_.data(), input_shape_.size());
            
            // Run inference
            auto outputs = session_->Run(
                Ort::RunOptions{nullptr},
                input_names_.data(), &input_tensor, 1,
                output_names_.data(), output_names_.size());
            
            // Extract results
            float* output_data = outputs[0].GetTensorMutableData<float>();
            waypoints.assign(output_data,
                           output_data + action_dim_);
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Inference error: %s", e.what());
            return false;
        }
    }
    
    bool is_ready() const override {
        return session_ != nullptr;
    }
};
```

### PostprocessorCore

**Purpose**: Interpolate low-frequency waypoints to smooth high-frequency control commands with safety constraints.

**Key Methods**:

```cpp
/// @brief Generate control commands from waypoints
/// @param[in] waypoints Target waypoints from policy (typically [-1, 1])
/// @param[out] commands Real robot-specific commands (e.g., positions)
/// @details Called at high frequency (1 kHz) by control loop.
///          Must be real-time safe - no allocations, system calls, or locks.
/// @note Waypoints and commands may have different coordinate systems.
///       This method performs any necessary conversions.
virtual void postprocess(const std::vector<double>& waypoints,
                         std::vector<double>& commands) = 0;

/// @brief Update with new trajectory waypoint
/// @param[in] waypoint New target waypoint from inference
/// @details Called when inference produces new waypoint (~50 Hz).
///          Starts new trajectory generation towards this waypoint.
virtual void update_waypoint(const std::vector<double>& waypoint) = 0;

/// @brief Get current robot state for trajectory planning
/// @param[out] current_position Current joint positions
/// @param[out] current_velocity Current joint velocities
/// @return true if state available
virtual bool get_current_state(std::vector<double>& current_position,
                               std::vector<double>& current_velocity) = 0;
```

**Usage with Trajectory Generation & Safety**:

```cpp
class TrajectoryPostprocessor : public PostprocessorCore {
private:
    MinimumJerkTrajectory trajectory_;
    SafetyConstraints safety_;
    std::vector<double> current_state_;
    std::vector<double> current_velocity_;
    double trajectory_duration_ = 1.0;  // seconds
    rclcpp::Time trajectory_start_time_;
    
public:
    void update_waypoint(const std::vector<double>& waypoint) override {
        // Denormalize waypoint from [-1, 1] to robot space
        std::vector<double> robot_waypoint = denormalize(waypoint);
        
        // Generate quintic trajectory from current position to waypoint
        trajectory_.compute(
            current_state_,
            robot_waypoint,
            trajectory_duration_);
        
        trajectory_start_time_ = now();
    }
    
    void postprocess(const std::vector<double>& waypoints,
                     std::vector<double>& commands) override {
        // Time since trajectory started
        double t = (now() - trajectory_start_time_).seconds();
        
        // Clamp at trajectory duration
        t = std::min(t, trajectory_duration_);
        
        // Evaluate trajectory at this timestep
        std::vector<double> pos, vel, acc;
        trajectory_.evaluate(t, pos, vel, acc);
        
        // Enforce safety constraints
        safety_.enforce_position_limits(pos);
        safety_.enforce_velocity_limits(vel);
        safety_.enforce_acceleration_limits(acc);
        
        commands = pos;  // Send positions to hardware
    }
    
    bool get_current_state(std::vector<double>& pos,
                          std::vector<double>& vel) override {
        pos = current_state_;
        vel = current_velocity_;
        return true;
    }
};
```

## Supporting Classes

### TrajectoryGenerator

**Purpose**: Base class for trajectory interpolation algorithms.

**Key Methods**:
```cpp
class TrajectoryGenerator {
public:
    virtual void compute(
        const std::vector<double>& start_position,
        const std::vector<double>& end_position,
        double duration) = 0;
    
    virtual void evaluate(
        double time,
        std::vector<double>& position,
        std::vector<double>& velocity,
        std::vector<double>& acceleration) = 0;
};
```

**Available Implementations**:
- `QuinticTrajectory`: C² continuous (position, velocity, acceleration continuous)
- `CubicTrajectory`: C¹ continuous (position, velocity continuous)
- `LinearTrajectory`: C⁰ continuous (position only, suitable for fast waypoint changes)

### SafetyConstraints

**Purpose**: Enforce joint and system-level safety limits.

**Key Methods**:
```cpp
class SafetyConstraints {
public:
    void set_position_limits(
        const std::vector<double>& min_pos,
        const std::vector<double>& max_pos) = 0;
    
    void set_velocity_limits(
        const std::vector<double>& max_vel) = 0;
    
    void set_acceleration_limits(
        const std::vector<double>& max_acc) = 0;
    
    void set_jerk_limits(
        const std::vector<double>& max_jerk) = 0;
    
    void enforce_limits(
        std::vector<double>& position,
        std::vector<double>& velocity,
        std::vector<double>& acceleration) = 0;
};
```

### RealtimeBuffer

**Purpose**: Lock-free communication between threads for real-time safety.

**Template Definition**:
```cpp
template <typename T>
class RealtimeBuffer {
public:
    /// Write new data (non-real-time thread)
    void writeFromNonRT(const T& data);
    
    /// Read latest data (real-time thread)
    bool readFromRT(T& data);
    
    /// Get const reference to current data
    const T* getReadDataPtr() const;
};
```

**Usage**:
```cpp
// In non-real-time inference thread:
RealtimeBuffer<std::vector<double>> waypoint_buffer;

void inference_thread() {
    while (running) {
        std::vector<double> waypoint = run_inference();
        waypoint_buffer.writeFromNonRT(waypoint);  // OK - no blocking
    }
}

// In real-time control loop (1 kHz):
return_type update() {
    std::vector<double> latest_waypoint;
    if (waypoint_buffer.readFromRT(latest_waypoint)) {
        // New waypoint available
        postprocessor->update_waypoint(latest_waypoint);
    }
    return_type::OK;
}
```

## Plugin Registration

### 1. Write Implementation

Create `my_robot_preprocessor.cpp`:
```cpp
#include "ai_control_bridge_core/preprocessor_core.hpp"

class MyRobotPreprocessor : public ai_control_bridge::PreprocessorCore {
    // ... implementation ...
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    my_namespace::MyRobotPreprocessor,
    ai_control_bridge::PreprocessorCore)
```

### 2. Create Plugin Description

In `my_robot_preprocessor_plugins.xml`:
```xml
<library path="my_robot_preprocessor">
    <class name="my_namespace/MyRobotPreprocessor"
           type="my_namespace::MyRobotPreprocessor"
           base_class_type="ai_control_bridge::PreprocessorCore">
        <description>
            Custom observation collector for my robot.
            Reads from my_robot's native sensor interfaces.
        </description>
    </class>
</library>
```

### 3. Export from CMake

In `CMakeLists.txt`:
```cmake
pluginlib_export_plugin_description_file(
    ai_control_bridge_core
    my_robot_preprocessor_plugins.xml)
```

### 4. Load at Runtime

In `ai_bridge_config.yaml`:
```yaml
preprocessor:
  type: "HardwareInterface"  # Use built-in
  # or:
  # type: "my_namespace/MyRobotPreprocessor"  # Use custom plugin
  
inference:
  type: "ONNXRuntime"
  model_path: "/path/to/policy.onnx"
  
postprocessor:
  type: "SafeQuinticTrajectory"
  trajectory_duration: 1.0
  max_velocity: [2.0, 2.0, 2.0]
  max_acceleration: [5.0, 5.0, 5.0]
```

## Real-Time Safety Requirements

### Thread Model

```
┌─────────────────────────────────────────────────────────┐
│  Main Program Thread                                     │
│  ├─ Loads configuration                                 │
│  ├─ Initializes plugins                                 │
│  └─ Launches control loop thread                        │
└─────────────────────────────────────────────────────────┘
                    ↓
    ┌───────────────────────────────────────┐
    │  Real-Time Control Loop Thread        │
    │  (1000 Hz, CPU-pinned, SCHED_FIFO)    │
    │                                        │
    │  1. Read observations (PreprocessorCore)
    │  2. Check for new waypoints            │
    │  3. Generate interpolated command     │
    │  4. Enforce safety constraints        │
    │  5. Send to hardware                  │
    │  └─ Max latency: 1ms                  │
    └───────────────────────────────────────┘
                    ↓
    ┌───────────────────────────────────────┐
    │  Inference Thread                      │
    │  (50 Hz, non-real-time)               │
    │                                        │
    │  1. Wait for new observations         │
    │  2. Run policy inference              │
    │  3. Publish waypoint (lock-free!)     │
    │  └─ Latency: ~20ms (non-critical)    │
    └───────────────────────────────────────┘
```

### Real-Time Safe Code Pattern

```cpp
// ✅ GOOD: Pre-sized stack allocation
std::array<double, 12> observations{};  // Fixed size
const double* obs_ptr = observations.data();

// ✅ GOOD: Bounded loop with fixed iteration count
for (size_t i = 0; i < num_joints_; ++i) {
    cmd[i] = compute_command(i);  // Bounded time
}

// ✅ GOOD: Lock-free buffer reads
if (wtf.readFromRT(waypoint)) {
    update_target(waypoint);  // Non-blocking
}

// ❌ BAD: Dynamic allocation in control loop
auto cache = std::make_unique<std::vector<double>>(100);  // Allocation!

// ❌ BAD: System calls
std::cout << "Debug output";  // Disk I/O!

// ❌ BAD: Unbounded algorithms
std::sort(data.begin(), data.end());  // Unpredictable time!

// ❌ BAD: Blocking synchronization
std::lock_guard<std::mutex> lock(mutex_);  // Can be preempted!
```

## Common Integration Patterns

### Pattern 1: Use Built-in Components

Minimum effort - everything provided:
```yaml
preprocessor:
  type: "HardwareInterface"       # Use ros2_control states
inference:
  type: "ONNXRuntime"             # Load ONNX model
  model_path: "model.onnx"
postprocessor:
  type: "SafeQuinticTrajectory"
```

### Pattern 2: Custom Preprocessing Only

For custom sensor fusion:
```cpp
class SensorFusionPreprocessor : public PreprocessorCore {
    // Fuse multiple sensors (IMU + force/torque + vision)
};
```

### Pattern 3: Custom Inference Only

For models not supported by built-in loaders:
```cpp
class TensorRTInference : public InferenceCore {
    // Run TensorFlow/PyTorch on GPU via TensorRT
};
```

### Pattern 4: Custom Safety/Postprocessing

For domain-specific constraints:
```cpp
class HumanoidSafetyPostprocessor : public PostprocessorCore {
    // Add contact-aware trajectory planning
    // Limit torques based on contact forces
    // Prevent joint locking
};
```

## Debugging

### Enable verbose logging:
```bash
export ROS_LOG_LEVEL=DEBUG
export RCUTILS_LOGGING_USE_STDOUT=1
ros2 launch ai_control_bridge_core ai_bridge_test.launch.py
```

### Monitor thread states:
```bash
# Check real-time thread priority
sudo chrt -p $(pgrep -f "ros2_control_node")
# Output: SCHED_FIFO priority 50

# Check thread CPU usage
top -p $(pgrep -f "ros2_control_node")
```

### Measure latency:
```cpp
auto start = std::chrono::high_resolution_clock::now();
postprocessor->postprocess(waypoints, commands);
auto elapsed =
    std::chrono::high_resolution_clock::now() - start;
auto ms = std::chrono::duration<double, std::milli>(elapsed).count();
if (ms > 1.0) {
    RCLCPP_WARN(get_logger(),
        "Postprocessor exceeded 1ms budget: %.3f ms", ms);
}
```

## References

- [PREEMPT_RT patching for real-time Linux](https://wiki.linuxfoundation.org/realtime/start)
- [ROS 2 Real-Time Middleware](https://docs.ros.org/en/rolling/Concepts/Advanced/Middleware.html)
- [ros2_control Controllers Guide](https://control.ros.org/master/doc/getting_started/getting_started.html)
- [Lock-Free Programming](https://www.1024cores.net/)

