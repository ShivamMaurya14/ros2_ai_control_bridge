# ai_control_bridge_core

Core abstract interfaces for the AI Control Bridge framework - a modular, extensible system for integrating learned robot policies with real-time hardware control.

## Overview

This package provides the foundation for the three-stage policy execution pipeline that decouples policy learning from hardware control:

1. **PreprocessorCore** (≤1 kHz) - Real-time observation collection and normalization
2. **InferenceCore** (~50 Hz) - Asynchronous neural network policy inference  
3. **PostprocessorCore** (≤1 kHz) - Real-time action interpolation and safety constraints

These abstract classes are **extension points** that allow developers to plug in custom implementations for any robot or sensor configuration. The framework handles threading, real-time safety, and ROS 2 integration automatically.

**Key Design Principle**: Separate concerns between high-frequency real-time loops and low-frequency inference, enabling smooth integration of learned policies with safety-critical control systems.

## Architecture

```
Three-Stage Policy Execution Pipeline
=====================================

Real Robot / Simulation
    ↓
[PreprocessorCore]  ← Collect observations (high freq, real-time)
    ↓
 observations[]
    ↓
[InferenceCore]     ← Run policy inference (~50 Hz, non-real-time)
    ↓
 waypoints[]
    ↓
[PostprocessorCore] ← Generate control commands (high freq, real-time)
    ↓
Control Commands
    ↓
Real Robot / Simulation
```

## Package Contents

### Headers

#### `preprocessor_core.hpp`
Abstract interface for observation collection.

**Key Method:**
```cpp
void collect_observations(std::vector<double>& observations);
```

**Responsibilities:**
- Gather sensor data from the robot
- Normalize/filter observations
- Manage observation frequency (typically 1 kHz)

**Example Implementation:**
```cpp
class MyPreprocessor : public PreprocessorCore {
public:
    void collect_observations(std::vector<double>& obs) override {
        // Read from joint state publishers
        obs.resize(12);  // 6 positions + 6 velocities
        obs[0] = joint1_position;
        obs[6] = joint1_velocity;
        // ... etc
    }
};
```

#### `inference_core.hpp`
Abstract interface for neural network policy inference.

**Key Method:**
```cpp
void infer(const std::vector<double>& observations,
           std::vector<double>& waypoints);
```

**Responsibilities:**
- Run pre-trained policy on observations
- Support multiple inference engines (ONNX, TensorRT, LibTorch, etc.)
- Execute at lower frequency (~50 Hz)
- Handle inference latency gracefully

**Example Implementation:**
```cpp
class ONNXInference : public InferenceCore {
private:
    Ort::Session session;
public:
    void infer(const std::vector<double>& obs,
               std::vector<double>& waypoints) override {
        // Run ONNX model
        auto output_tensors = session.Run(input_tensors);
        waypoints = extract_waypoints(output_tensors);
    }
};
```

#### `postprocessor_core.hpp`
Abstract interface for action postprocessing and safety.

**Key Method:**
```cpp
void postprocess(const std::vector<double>& waypoints,
                 std::vector<double>& commands);
```

**Responsibilities:**
- Interpolate waypoints to control frequency
- Enforce safety constraints
- Generate real-time commands
- Handle non-real-time latency

**Example Implementation:**
```cpp
class SafePostprocessor : public PostprocessorCore {
private:
    MinimumJerkTrajectory trajectory;
    SafetyFilter safety;
public:
    void postprocess(const std::vector<double>& waypoints,
                     std::vector<double>& commands) override {
        // Generate smooth trajectory
        trajectory.generate_quintic(waypoints);
        
        // Apply safety constraints
        safety.enforce_limits(trajectory.get_point(), commands);
    }
};
```

### Supporting Headers

#### `realtime_buffer.hpp`
Template for lock-free, real-time safe communication between stages.

#### `trajectory_generator.hpp`
Base class for trajectory generation algorithms.

#### `safety_constraints.hpp`
Base class for safety constraint enforcement.

#### `controller_bridge.hpp`
Base class for ros2_control integration.

## Dependencies

```
rclcpp           - ROS 2 client library
rclcpp_lifecycle - Lifecycle node management
pluginlib        - Plugin system for extensibility
```

## Usage

### 1. Add as Dependency

In your `package.xml`:
```xml
<build_depend>ai_control_bridge_core</build_depend>
<exec_depend>ai_control_bridge_core</exec_depend>
```

In your `CMakeLists.txt`:
```cmake
find_package(ai_control_bridge_core REQUIRED)
target_link_libraries(my_target ai_control_bridge_core::ai_control_bridge_core)
```

### 2. Implement an Extension

```cpp
#include "ai_control_bridge_core/preprocessor_core.hpp"

class MyObservationCollector : public ai_control_bridge::PreprocessorCore {
public:
    void collect_observations(std::vector<double>& obs) override {
        // Your implementation here
    }
};
```

### 3. Register Plugin

In your `my_preprocessor_plugin.xml`:
```xml
<library path="my_preprocessor_lib">
  <class name="my_namespace::MyObservationCollector"
         type="my_namespace::MyObservationCollector"
         base_class_type="ai_control_bridge::PreprocessorCore">
    <description>Custom observation collector implementation</description>
  </class>
</library>
```

### 4. Export Plugin

In your `CMakeLists.txt`:
```cmake
pluginlib_export_plugin_description_file(
    ai_control_bridge_core
    my_preprocessor_plugin.xml
)
```

## Real-Time Considerations

All implementations should follow these guidelines:

### For PreprocessorCore & PostprocessorCore
✅ **Must be real-time safe** (typically 1 kHz):
- No dynamic memory allocations
- No blocking operations
- Use lock-free buffers for data exchange
- Bounded loop times

### For InferenceCore
⚠️ **Non-real-time but bounded**:
- May block (e.g., model inference)
- Should execute asynchronously
- Use bounded buffers to handle latency
- Typical frequency: 10-50 Hz

## Performance Targets

| Component | Frequency | Latency | Real-Time |
|-----------|-----------|---------|-----------|
| PreprocessorCore | 1000 Hz | <1 ms | ✅ Yes |
| InferenceCore | 50 Hz | 20 ms | ❌ No |
| PostprocessorCore | 1000 Hz | <0.5 ms | ✅ Yes |

## Testing

To test your implementations:

```cpp
#include <gtest/gtest.h>
#include "ai_control_bridge_core/preprocessor_core.hpp"

TEST(MyPreprocessor, CollectsObservations) {
    MyPreprocessor preprocessor;
    std::vector<double> obs;
    
    preprocessor.collect_observations(obs);
    
    EXPECT_EQ(obs.size(), 12);  // 6 position + 6 velocity
    EXPECT_TRUE(std::all_of(obs.begin(), obs.end(),
                           [](double v) { return !std::isnan(v); }));
}
```

## Common Patterns

### Pattern 1: Multi-Stage Pipeline with Buffers
```cpp
PreprocessorCore obs_collector;
InferenceCore policy;
PostprocessorCore command_gen;

// In real-time controller loop (1 kHz)
obs_collector.collect_observations(current_obs);
command_gen.postprocess(latest_waypoints, commands);

// In inference thread (~50 Hz, asynchronous)
policy.infer(latest_obs, waypoints);
```

### Pattern 2: Custom Constraints
Extend `SafetyConstraints` to add robot-specific limits:
```cpp
class MyRobotSafety : public SafetyConstraints {
public:
    void enforce(std::vector<double>& commands) override {
        // Apply robot-specific safety rules
        for (size_t i = 0; i < commands.size(); ++i) {
            commands[i] = std::clamp(
                commands[i],
                min_limits_[i],
                max_limits_[i]
            );
        }
    }
};
```

## Extending the Framework

### Add a New Stage
1. Create new abstract class in `include/ai_control_bridge_core/`
2. Define clear interface with pure virtual methods
3. Document expected behavior and frequency
4. Update root `package.xml` and `CMakeLists.txt`

### Integrate New Inference Engine
1. Inherit from `InferenceCore`
2. Implement `infer()` method using desired library (ONNX, TensorRT, etc.)
3. Register via plugin system
4. See implementation in `ai_inference_processor` package

## Troubleshooting

**Problem:** Plugin not loading
- Verify plugin.xml is in share directory: `ros2 pkg prefix <package>`
- Check `pluginlib_export_plugin_description_file()` in CMakeLists.txt

**Problem:** Real-time violations
- Use `top -p $(pidof ros2_control_node)` to monitor CPU usage
- Check for dynamic allocations with `new` or `malloc`
- Use `std::vector::reserve()` to pre-allocate buffers

**Problem:** Inference latency too high
- Profile with `ros2 run ai_inference_processor inference_processor` 
- Check model size and input dimensions
- Consider model quantization or pruning

## API Reference

### PreprocessorCore
```cpp
class PreprocessorCore {
public:
    virtual ~PreprocessorCore() = default;
    virtual void collect_observations(std::vector<double>& obs) = 0;
};
```

### InferenceCore
```cpp
class InferenceCore {
public:
    virtual ~InferenceCore() = default;
    virtual void infer(const std::vector<double>& obs,
                       std::vector<double>& waypoints) = 0;
};
```

### PostprocessorCore
```cpp
class PostprocessorCore {
public:
    virtual ~PostprocessorCore() = default;
    virtual void postprocess(const std::vector<double>& waypoints,
                             std::vector<double>& commands) = 0;
};
```

## License

Apache License 2.0 - See LICENSE in root directory

## References

- [pluginlib Documentation](http://wiki.ros.org/pluginlib)
- [ROS 2 Real-time Guidelines](https://docs.ros.org/en/humble/Tutorials/Advanced/Real-Time-Programming.html)
- [ros2_control Framework](https://control.ros.org/)

## Support

For issues and questions, see main project repository issues tracker.

