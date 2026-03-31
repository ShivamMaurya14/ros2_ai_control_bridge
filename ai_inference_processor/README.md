# ai_inference_processor

ROS 2 node implementing InferenceCore interface for running pre-trained neural network policies at ~50 Hz. **Works with any robot.**

## Overview

This package provides a concrete, generalizable implementation of the **InferenceCore** interface. It runs pre-trained control policies (trained with PyTorch, TensorFlow, JAX, etc.) and converts sensor observations into low-frequency waypoint sequences for the real-time control bridge.

**Applicable to:** Any robot with learned policies - manipulators, humanoids, wheeled robots, aerial vehicles, etc.

### Key Features

- 📊 **Non-Real-Time**: Decoupled from real-time loop (~50 Hz, configurable)
- 🧠 **Framework Agnostic**: Supports ONNX, TensorRT, LibTorch, PyTorch, TensorFlow, etc.
- 🔄 **Async Design**: Won't block real-time controller if inference lags
- 📦 **Pre-Trained Models**: Works with models exported from any ML framework
- 🛡️ **Safe Fallback**: Gracefully handles inference failures and timeouts
- 🤖 **Robot-Agnostic**: Works with any DOF, any observation/action space

## System Integration

```
Real Robot / Simulation (1 kHz)
          ↓
   [Preprocessor]  ← Observations (high freq)
          ↓
   observations[]
          ↓
┌─────────────────────────────┐
│ ai_inference_processor node │  (~50 Hz, non-real-time)
│  (This package)             │
│ - Load ONNX model           │
│ - Run inference             │
│ - Publish waypoints         │
└─────────────────────────────┘
          ↓
   waypoints[]
          ↓
   [Postprocessor]  ← Commands (high freq, real-time safe)
          ↓
Joint Commands → Robot Hardware
```

## Architecture

### Node Structure

```
InferenceProcessor (ROS 2 Node)
├── Parameters
│   ├── model_path: Path to ONNX/TensorRT model
│   ├── inference_frequency: ~50 Hz (configurable)
│   ├── max_inference_time: 15 ms (timeout)
│   └── input_normalization: Obs mean/std
│
├── Subscriptions
│   └── /observations (std_msgs/Float64MultiArray)
│       Input from PreprocessorCore
│
├── Publishers
│   └── /waypoints (std_msgs/Float64MultiArray)
│       Output for PostprocessorCore
│
└── Inference Engine
    ├── ONNX Runtime
    ├── TensorRT (GPU acceleration)
    ├── LibTorch (C++ first-class support)
    └── Custom backends
```

## Package Structure

```
ai_inference_processor/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # This file
├── include/
│   └── ai_inference_processor/
│       └── (Header files)
└── src/
    └── inference_processor.cpp # Main node implementation
```

## Configuration

This package is typically launched as part of the AI Control Bridge system. Configuration is managed through the launch system in `ai_control_bridge_controller`.

### Key Parameters

- `model_path` - Path to ONNX/TensorRT neural network model
- `observation_dim` - Input dimension (typically observation vector size)
- `waypoint_dim` - Output dimension (typically number of joints)
- `inference_frequency` - Target inference frequency (~50 Hz recommended)
- `max_inference_time` - Maximum time allowed for inference (ms)

## Usage

This node is typically started by the `ai_control_bridge_controller` launch system. To launch directly:

```bash
ros2 run ai_inference_processor inference_processor \
  --ros-args -p model_path:=/path/to/policy.onnx \
             -p observation_dim:=12 \
             -p waypoint_dim:=6 \
             -p inference_frequency:=50.0
```

# Option B: Run node directly
ros2 run ai_inference_processor inference_processor \
  --ros-args -p model_path:=/path/to/policy.onnx
```

### 5. Monitor Inference

```bash
# View published waypoints
ros2 topic echo /waypoints

# View inference statistics (if enabled)
ros2 topic echo /inference_stats

# Monitor node status
ros2 node info /ai_inference_processor
```

## Topics

### Subscriptions

**`/observations`** - `std_msgs/msg/Float64MultiArray`
- **Source**: PreprocessorCore implementation
- **Frequency**: High (1 kHz typical)
- **Message Format**:
  ```
  data: [joint1_pos, joint2_pos, ..., joint1_vel, joint2_vel, ...]
  ```
- **Purpose**: Latest observation for policy inference

### Publications

**`/waypoints`** - `std_msgs/msg/Float64MultiArray`
- **Destination**: PostprocessorCore (controller bridge)
- **Frequency**: ~50 Hz (configurable)
- **Message Format**:
  ```
  data: [wp1_joint1, wp1_joint2, ..., wp2_joint1, wp2_joint2, ...]
  ```
- **Latency**: ~20 ms for inference + publishing

**`/inference_stats`** - `std_msgs/msg/Float64MultiArray` (optional)
- **Frequency**: ~50 Hz
- **Content**: [inference_time_ms, model_exec_time_ms, pub_time_ms]

## Implementation Details

### Inference Loop

```cpp
void InferenceProcessor::run_inference() {
    while (rclcpp::ok()) {
        // 1. Wait for latest observation (non-blocking)
        auto obs = get_latest_observation();
        
        if (!obs) continue;
        
        // 2. Normalize if needed
        normalized_obs = normalize(obs);
        
        // 3. Run inference (with timeout)
        auto start = now();
        waypoints = model->forward(normalized_obs);
        auto inference_time = (now() - start).count();
        
        if (inference_time > max_inference_time) {
            RCLCPP_WARN(get_logger(), 
                "Inference took %.2f ms (limit: %.2f ms)",
                inference_time, max_inference_time);
        }
        
        // 4. Publish waypoints
        publish_waypoints(waypoints);
        
        // 5. Sleep to maintain target frequency
        sleep_until_next_inference();
    }
}
```

### Error Handling

```cpp
try {
    waypoints = model->infer(observations);
} catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Inference failed: %s", e.what());
    
    if (use_last_valid_) {
        // Republish last known good waypoints
        waypoints = last_valid_waypoints_;
    } else {
        // Zero waypoints (safe default)
        waypoints = std::vector<double>(waypoint_dim, 0.0);
    }
}
```

## Supported Inference Backends

### ONNX Runtime (Recommended for CPU)

```cpp
#include <onnxruntime_cxx_api.h>

Ort::SessionOptions session_options;
session_options.SetGraphOptimizationLevel(
    GraphOptimizationLevel::ORT_ENABLE_ALL);

Ort::Session session(env, model_path.c_str(), session_options);
```

**Pros:**
- Cross-platform (CPU, GPU, Mobile)
- Excellent performance
- Model framework agnostic

**Cons:**
- GPU support via separate runtime

### TensorRT (For NVIDIA GPU)

```cpp
#include <NvInferRuntime.h>

auto engine = trt_runtime.deserializeCudaEngine(
    engine_data, engine_size, pluginFactory);
auto context = engine->createExecutionContext();
```

**Pros:**
- Maximum GPU performance
- Production-ready

**Cons:**
- NVIDIA GPU required
- Model conversion needed

### LibTorch (For PyTorch Models)

```cpp
#include <torch/script.h>

auto module = torch::jit::load(model_path);
auto output = module.forward({input_tensor});
```

**Pros:**
- Direct PyTorch support
- Easy model debugging

**Cons:**
- Larger binary size

## Performance

### Typical Metrics

| Backend | Model Size | Inference Time | Memory | GPU Required |
|---------|-----------|-----------------|--------|--------------|
| ONNX CPU | ~50 MB | 15-20 ms | ~200 MB | ❌ |
| ONNX GPU | ~50 MB | 2-5 ms | ~500 MB | ✅ |
| TensorRT | ~30 MB | 1-3 ms | ~600 MB | ✅ |
| LibTorch | ~100 MB | 10-15 ms | ~300 MB | ❌ |

### Optimization Tips

1. **Model Quantization**
   ```python
   # PyTorch dynamic quantization
   quantized_model = torch.quantization.quantize_dynamic(
       model, {torch.nn.Linear}, dtype=torch.qint8)
   ```

2. **Batch Normalization Folding** (automatic in ONNX)

3. **Operator Fusion** (automatic in ONNX, manual in TensorRT)

4. **Input Caching** - Store last observation, avoid unnecessary copies

## Extending the Node

### Add Custom Preprocessing

```cpp
class MyInferenceProcessor : public InferenceProcessor {
protected:
    std::vector<double> preprocess(
        const std::vector<double>& obs) override {
        
        // Add custom preprocessing
        auto processed = normalize(obs);
        apply_my_filter(processed);
        return processed;
    }
};
```

### Add Custom Postprocessing

```cpp
class MyInferenceProcessor : public InferenceProcessor {
protected:
    std::vector<double> postprocess(
        const std::vector<double>& waypoints) override {
        
        // Add custom postprocessing
        auto smooth = smooth_waypoints(waypoints);
        clip_to_limits(smooth);
        return smooth;
    }
};
```

### Add Custom Inference Timeout Strategy

```cpp
// In config, set use_last_valid: true
// Node automatically republishes last valid waypoints
// if current inference times out
```

## Troubleshooting

### No Waypoints Published

```bash
# Check observations are being published
ros2 topic echo /observations

# Check node is running
ros2 node list | grep inference

# Check for errors
ros2 node info /ai_inference_processor
```

### High Inference Latency

**Diagnosis:**
```bash
# Check timing stats
ros2 topic echo /inference_stats

# Profile with top
top -p $(pgrep inference_processor_node)
```

**Solutions:**
1. Check model is on GPU:
   ```yaml
   model_type: "tensorrt"  # vs "onnx"
   ```

2. Reduce model complexity:
   ```python
   # Prune or quantize model
   torch.quantization.quantize_dynamic(model, ...)
   ```

3. Reduce observation dimension if possible

### Model Loading Failed

```bash
# Verify model path
file /path/to/policy.onnx

# Check ONNX model validity
python3 -c "import onnx; onnx.checker.check_model('/path/to/policy.onnx')"
```

## Testing

### Unit Test

```cpp
#include <gtest/gtest.h>
#include "ai_inference_processor/inference_processor.hpp"

TEST(InferenceProcessor, ProcessesObservations) {
    auto processor = std::make_shared<InferenceProcessor>();
    
    std::vector<double> obs(12, 0.1);  // 12-dim observation
    auto waypoints = processor->infer(obs);
    
    EXPECT_EQ(waypoints.size(), 30);  // 6-dim * 5 waypoints
}
```

### Integration Test

```bash
# Terminal 1: Start your robot (simulator or real hardware)
# ros2 launch your_robot_bringup robot.launch.py

# Terminal 2: Start preprocessor to collect observations
ros2 run ai_inference_processor dummy_preprocessor

# Terminal 3: Start inference processor
ros2 launch ai_inference_processor inference_processor.launch.py

# Terminal 4: Monitor waypoints being published
ros2 topic echo /waypoints
```

## References

- [ONNX Runtime](https://onnxruntime.ai/)
- [TensorRT](https://developer.nvidia.com/tensorrt)
- [PyTorch TorchScript](https://pytorch.org/docs/stable/jit.html)
- [ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

## License

Apache License 2.0 - See LICENSE in root directory

## Support

For issues:
- **Core pipeline**: See `ai_control_bridge_core/README.md`
- **Controller**: See `ai_control_bridge_controller/README.md`
- **Hardware**: See `ai_control_bridge_hardware/README.md`
