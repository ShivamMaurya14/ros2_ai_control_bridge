# Model Integration Guide - AI Control Bridge

Complete guide to train, export, and integrate neural network policies with the AI Control Bridge.

## Overview

The AI Control Bridge accepts pre-trained neural network policies in multiple formats:
- ONNX (recommended - framework agnostic)
- TensorRT (optimized for NVIDIA GPUs)
- LibTorch (PyTorch C++ runtime)
- TensorFlow (via TF-Lite or SavedModel)

This guide focuses on **ONNX export** (most portable and recommended).

## Training Your Policy

### 1. Using PyTorch (Recommended)

```python
import torch
import torch.nn as nn

class PolicyNetwork(nn.Module):
    """Neural network policy for robot control."""
    
    def __init__(self, obs_dim=12, action_dim=6, hidden_dim=256):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()  # Output in [-1, 1] range
        )
    
    def forward(self, observations):
        return self.network(observations)

# Train your policy (pseudocode)
model = PolicyNetwork(obs_dim=12, action_dim=6)

optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
for epoch in range(1000):
    for obs_batch, target_action in training_data:
        pred_action = model(obs_batch)
        loss = nn.MSELoss()(pred_action, target_action)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

print(f"Training complete. Loss: {loss.item():.4f}")

# Save checkpoint
torch.save(model.state_dict(), 'policy_checkpoint.pt')
```

### 2. Using TensorFlow/Keras

```python
import tensorflow as tf

# Build model
model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(12,)),
    tf.keras.layers.Dense(256, activation='relu'),
    tf.keras.layers.Dense(256, activation='relu'),
    tf.keras.layers.Dense(6, activation='tanh'),
])

# Train
model.compile(optimizer='adam', loss='mse')
model.fit(obs_data, action_data, epochs=100, batch_size=32)

# Save
model.save('policy_model')
```

### 3. Using JAX

```python
import jax
import jax.numpy as jnp
from flax import linen as nn

class JAXPolicy(nn.Module):
    @nn.compact
    def __call__(self, x):
        x = nn.Dense(256)(x)
        x = nn.relu(x)
        x = nn.Dense(256)(x)
        x = nn.relu(x)
        x = nn.Dense(6)(x)
        return jnp.tanh(x)

# Train and save model
```

## Exporting to ONNX

### PyTorch to ONNX

```python
import torch
import torch.onnx

# Load trained model
model = PolicyNetwork(obs_dim=12, action_dim=6)
model.load_state_dict(torch.load('policy_checkpoint.pt'))
model.eval()  # Set to evaluation mode

# Create dummy input (batch_size=1, obs_dim=12)
dummy_input = torch.randn(1, 12)

# Export to ONNX
torch.onnx.export(
    model,
    dummy_input,
    'policy.onnx',
    input_names=['observations'],
    output_names=['waypoints'],
    opset_version=12,  # ONNX opset version (12 is widely supported)
    do_constant_folding=True,
    verbose=True
)

print("✓ Model exported to policy.onnx")

# Verify ONNX model
import onnx
onnx_model = onnx.load('policy.onnx')
onnx.checker.check_model(onnx_model)
print("✓ ONNX model is valid")
```

### TensorFlow to ONNX

```python
# Using tf2onnx
import tf2onnx
import tensorflow as tf

model = tf.keras.models.load_model('policy_model')

# Convert
spec = (tf.TensorSpec((None, 12), tf.float32, name="observations"),)
output_path = "policy.onnx"

model_proto, _ = tf2onnx.convert.from_keras(model, input_signature=spec)
with open(output_path, "wb") as f:
    f.write(model_proto.SerializeToString())

print(f"✓ Model exported to {output_path}")
```

## Verifying ONNX Export

```python
import onnx
import onnxruntime as ort
import numpy as np

# Load and check ONNX model
onnx_model = onnx.load('policy.onnx')
onnx.checker.check_model(onnx_model)
print("✓ ONNX model structure valid")

# Print model information
print("\nModel Info:")
print(f"  Graph inputs: {[input.name for input in onnx_model.graph.input]}")
print(f"  Graph outputs: {[output.name for output in onnx_model.graph.output]}")

# Test inference with ONNX Runtime
session = ort.InferenceSession('policy.onnx', providers=['CPUExecutionProvider'])

# Run test inference
test_input = np.random.randn(1, 12).astype(np.float32)
outputs = session.run(None, {'observations': test_input})

print(f"\n✓ Test inference successful")
print(f"  Input shape: {test_input.shape}")
print(f"  Output shape: {outputs[0].shape}")
print(f"  Output sample: {outputs[0][0, :3]}...")  # Print first 3 actions
```

## Quantization (Optional - for Faster Inference)

### ONNX Dynamic Quantization

Reduces model size and improves inference speed on CPU:

```python
from onnxruntime.quantization import quantize_dynamic, QuantType

# Quantize FP32 to INT8
quantize_dynamic(
    'policy.onnx',
    'policy_quantized.onnx',
    weight_type=QuantType.QInt8  # 8-bit integer quantization
)

print("✓ Model quantized: policy_quantized.onnx")
print("  Benefits: ~4x smaller, ~2x faster on CPU")
```

### TensorRT Optimization (NVIDIA GPU)

For GPU deployment:

```bash
# Install TensorRT
pip install tensorrt

# Convert ONNX to TensorRT
python -c "
import tensorrt as trt
import onnx

# Create logger
logger = trt.Logger(trt.Logger.INFO)
builder = trt.Builder(logger)

# Load ONNX model
with open('policy.onnx', 'rb') as f:
    network = builder.network
    # ... TensorRT conversion code ...

print('✓ Model optimized for GPU')
"
```

## Deployment: Placing Model File

### 1. Directory Structure

```
your_project/
├── models/
│   ├── policy.onnx              # Main model
│   ├── policy_quantized.onnx    # Quantized version (fast CPU)
│   └── README_models.md         # Model documentation
├── config/
│   └── inference_config.yaml    # Inference parameters
└── ...
```

### 2. Update Configuration

In `config/inference_config.yaml`:

```yaml
ai_inference_processor:
  ros__parameters:
    # Absolute path
    model_path: "/home/user/models/policy.onnx"
    
    # OR relative path (from ROS_PACKAGE_PATH)
    model_path: "${ai_inference_processor_DIR}/../models/policy.onnx"
    
    # Model format
    model_type: "onnx"  # onnx, tensorrt, libtorch
    
    # Input/output dimensions MUST match your model
    observation_dim: 12
    action_dim: 6
    
    # Normalization (if your model expects normalized inputs)
    normalize_inputs: true
    input_mean: [0.0, 0.0, 0.0, ...]  # From training dataset
    input_std: [1.0, 1.0, 1.0, ...]   # From training dataset
```

### 3. Compute Normalization Statistics

```python
import numpy as np

# Load your training observations
observations = np.load('training_observations.npy')

# Compute mean and std
mean = np.mean(observations, axis=0)
std = np.std(observations, axis=0)

print(f"input_mean: {mean.tolist()}")
print(f"input_std: {std.tolist()}")
```

## Runtime Integration

### Manual Inference Test

```bash
# Terminal 1: Start inference node
ros2 run ai_inference_processor inference_processor_node \
  --ros-args -p model_path:=/path/to/policy.onnx

# Terminal 2: Test with dummy observations
ros2 run std_msgs Float64MultiArray --message="data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
  -t /observations

# Terminal 3: Watch output waypoints
ros2 topic echo /waypoints
```

### Automated Launch

```bash
# Full system launch
ros2 launch ai_control_bridge_hardware example_robot_bringup.launch.py
```

## Performance Optimization

### Benchmark Your Model

```python
import time
import numpy as np
import onnxruntime as ort

session = ort.InferenceSession('policy.onnx')

# Warmup
for _ in range(10):
    session.run(None, {'observations': np.random.randn(1, 12).astype(np.float32)})

# Benchmark
times = []
for _ in range(1000):
    start = time.time()
    session.run(None, {'observations': np.random.randn(1, 12).astype(np.float32)})
    times.append((time.time() - start) * 1000)  # Convert to ms

print(f"Inference Time (ms):")
print(f"  Mean: {np.mean(times):.2f}")
print(f"  Median: {np.median(times):.2f}")
print(f"  Max: {np.max(times):.2f}")
print(f"  Min: {np.min(times):.2f}")

# For 50 Hz target (20 ms between inferences)
if np.mean(times) < 15:
    print("✓ Model meets 50 Hz inference target (20 ms)")
else:
    print(f"✗ Model too slow: {np.mean(times):.1f} ms > 15 ms target")
    print("  Options: quantize model, reduce hidden layer size, or increase inference frequency allowance")
```

## Troubleshooting

### Issue: "Model not found"
- **Check path**: Ensure model_path in config points to correct location
- **ROS paths**: Use `${<package>_DIR}` for relative paths
- **Absolute path**: Easier to debug (though less portable)

### Issue: "Input/output dimension mismatch"
- **Verify**: Run test inference script above
- **Check config**: observation_dim and action_dim must match model
- **Model inspection**: Use `onnx.load()` and print input shapes

### Issue: "Inference too slow"
- **Benchmark**: Run performance test above
- **Optimize**:
  1. Reduce model size (fewer layers/units)
  2. Quantize to INT8
  3. Use TensorRT on GPU (if available)
  4. Reduce inference frequency requirement

### Issue: "Poor control quality"
- **Verify** model is trained correctly
- **Check normalization**: Mean/std should match training
- **Test offline**: Run model on saved test trajectories
- **Tune frequency**: Maybe policy needs more frequent updates

## Advanced: Custom Inference Backend

Implement your own `InferenceCore`:

```cpp
#include "ai_control_bridge_core/inference_core.hpp"

class CustomInference : public ai_control_bridge::InferenceCore {
    bool infer(const std::vector<double>& obs,
               std::vector<double>& waypoints) override {
        // Your custom inference logic
        # Could be:
        # - TensorFlow C++ API
        # - Custom CUDA kernel
        # - RPC to inference server
        # - Anything!
    }
};

PLUGINLIB_EXPORT_CLASS(CustomInference, InferenceCore)
```

## References

- [ONNX Documentation](https://onnx.ai/)
- [ONNX Runtime](https://github.com/microsoft/onnxruntime)
- [PyTorch ONNX Export](https://pytorch.org/docs/stable/onnx.html)
- [TensorFlow tf2onnx](https://github.com/onnx/tensorflow-onnx)
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/)

---

**Happy training and deploying!** 🚀
