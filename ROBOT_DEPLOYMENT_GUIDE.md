# AI Control Bridge - Robot Deployment Guide

**Status**: Production Ready for Physical Hardware Testing  
**Date**: March 31, 2026  
**Target Robot**: BCR ARM (7-DOF Collaborative Robot Arm)

---

## Table of Contents
1. [Pre-Deployment Checklist](#pre-deployment-checklist)
2. [Hardware Setup](#hardware-setup)
3. [Software Installation](#software-installation)
4. [Calibration Procedure](#calibration-procedure)
5. [Deployment Steps](#deployment-steps)
6. [Testing & Validation](#testing--validation)
7. [Troubleshooting](#troubleshooting)
8. [Safety Guidelines](#safety-guidelines)

---

## Pre-Deployment Checklist

### Hardware Requirements
- [ ] BCR ARM robot arm (7 DOF revolute joints)
- [ ] Controller hardware (PC/Industrial PC with ROS 2 compatible OS)
- [ ] Real-time compatible OS: Ubuntu 22.04 with RT kernel
- [ ] Network: Ethernet to robot controller (GigE or faster)
- [ ] Power supply for arm and controller
- [ ] Emergency stop button (wired to controller)
- [ ] Joint encoders/sensors for state feedback
- [ ] Actuators capable of position control

### Software Requirements
- [ ] ROS 2 Jazzy installed and tested
- [ ] ros2_control framework configured
- [ ] AI Control Bridge packages built (all 9 packages)
- [ ] BCR ARM description package updated
- [ ] Python 3.10+ installed
- [ ] ONNX Runtime or similar for model inference

### Team Requirements
- [ ] 2+ trained operators (for safety)
- [ ] 1 systems engineer (setup)
- [ ] Robot workspace cleared of obstacles
- [ ] Safety enclosure in place
- [ ] Emergency procedures reviewed

---

## Hardware Setup

### Step 1: Verify Controller Connection

```bash
# Check if hardware interface is reachable
ros2 node list
ros2 topic list

# Check available control interfaces
ros2 control list_controllers
```

**Expected Output**:
- No errors
- Joint state broadcasters listed
- Control interfaces visible

### Step 2: Configure Communication

Edit `ai_control_bridge_controller/config/bcr_arm_real.yaml`:

```yaml
robot:
  name: "bcr_arm"
  namespace: "bcr_arm"

# Hardware communication parameters
hardware:
  interface: "eth0"
  baudrate: 1000000  # 1 Mb/s
  timeout: 100  # milliseconds
```

### Step 3: Verify Joint Interfaces

```bash
# Inspect available controllers
ros2 control list_controllers

# Expected output includes:
# - ai_bridge_controller (active)
# - joint_state_broadcaster (active)
```

---

## Software Installation

### Step 1: Clone and Build

```bash
cd ~/GSoC
git clone <repo_url> .
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Step 2: Install Python Dependencies

```bash
pip install onnxruntime  # or tensorrt if available
pip install pyyaml
pip install diagnostic_aggregator
```

### Step 3: Setup ROS 2 Control Environment

```bash
# Setup real-time kernel (if not already done)
sudo apt install linux-image-rt-generic

# Create control group for real-time priority
sudo usermod -a -G sudo $USER
# Create realtime group
sudo groupadd realtime
sudo usermod -a -G realtime $USER
```

### Step 4: Copy Configuration Files

```bash
# Copy real hardware config
cp config/bcr_arm_real.yaml ~/.local/share/ai_control_bridge/config/

# Update paths in config to match your system
# Edit: model path, home position, etc.
```

---

## Calibration Procedure

**Time Required**: 15-30 minutes  
**Safety**: Must be done with emergency stop armed

### Step 1: Launch Calibration Utility

```bash
source install/setup.bash
ros2 run ai_control_bridge_core calibration_utility --config bcr_arm_real.yaml
```

### Step 2: Follow Interactive Wizard

The script will guide you through:

1. **Encoder Offset Calibration**
   - Move each joint to mechanical home (0 radians)
   - Script records current encoder readings
   - Offsets are automatically calculated
   
   ```
   CALIBRATION: ENCODER OFFSETS
   ============================================================
   INSTRUCTIONS:
   1. Manually move each joint to its mechanical HOME position
   2. For revolute: typically centered (0 radians)
   3. For prismatic: typically fully retracted
   
   Press ENTER when all joints are at home position...
   ```

2. **Gravity Compensation (Optional)**
   - For accurate quasi-static operation
   - Requires force/torque sensors
   - Can be skipped for now

3. **Joint Limit Verification**
   - Move each joint to minimum position
   - Verify no collision or mechanical stress
   - Move each joint to maximum position
   - Verify again

4. **Home Position Test**
   - Robot moves to all-zeros configuration
   - Verifies this position is reachable
   - Confirms calibration is correct

### Step 3: Verify Calibration

```bash
# Check updated config file
cat ~/.local/share/ai_control_bridge/config/bcr_arm_real.yaml | grep calibration_offset

# Move to home and verify positions
ros2 service call /move_to_home
```

---

## Deployment Steps

### Step 1: Pre-Flight Checks

```bash
# Terminal 1: Check system health
ros2 run ai_control_bridge_core diagnostics_node --config bcr_arm_real.yaml
# Should show: OK status for all components
```

### Step 2: Launch Main System

```bash
# Terminal 2: Launch real robot deployment
source install/setup.bash
ros2 launch ai_control_bridge_controller ai_control_bridge_real_robot.launch.py \
  robot_config:=bcr_arm_real.yaml \
  launch_rviz:=true \
  verbose:=false
```

**Expected startup sequence**:
1. Hardware interface connects (~2 seconds)
2. Joint state publisher starts
3. Control loop activates (1000 Hz)
4. Inference processor starts (50 Hz)
5. RViz launches showing robot model
6. Diagnostics begin reporting

### Step 3: Run Neural Network Model

```bash
# Terminal 3: Verify model loads correctly
ros2 run ai_inference_processor ai_inference_processor_node \
  --config bcr_arm_real.yaml
```

Monitor output:
```
[INFO] Model loaded: /path/to/policy.onnx
[INFO] Input shape: (1, 7)  # 7 joints
[INFO] Output shape: (1, 7)  # 7 waypoints
[INFO] Inference thread started @ 50 Hz
```

### Step 4: Enable Waypoint Commands

```bash
# Terminal 4: Send test waypoint
# This will be the AI policy output once inference is connected
# For now, send dummy test waypoint
ros2 topic pub /bcr_arm/waypoints std_msgs/Float64MultiArray \
  "data: [0.1, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0]"
```

Watch robot arm:
- Should start moving smoothly toward waypoint
- Motion should respect acceleration limits
- Safety constraints should be enforced

---

## Testing & Validation

### Test 1: Control Loop Frequency

```bash
# Check actual control frequency (should be 1000 Hz)
ros2 run diagnostic_aggregator aggregator_node

# Access via web UI:
# http://localhost:8085
```

### Test 2: Constraint Enforcement

```bash
# Send command exceeding velocity limit
ros2 topic pub /bcr_arm/waypoints std_msgs/Float64MultiArray \
  "data: [3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14]"
  
# Robot should slow down to respect limits
# Check diagnostics: /constraints_violations should remain at 0
```

### Test 3: Trajectory Smoothness

```bash
# Record joint states while moving to waypoint
ros2 bag record -o test_motion /joint_states

# Analyze trajectory smoothness
# Plot position, velocity, acceleration
# Should show smooth quintic profile with no jerky motions
```

### Test 4: Emergency Stop

```bash
# Verify e-stop works
# Press physical e-stop button
# All robot motion should halt immediately
# Check /diagnostics: should show ERROR state

# After e-stop is reset:
ros2 service call /controllers/reset_safety
```

### Test 5: Inference Integration

```bash
# Verify inference pipeline is connected
ros2 topic list | grep waypoints
ros2 topic list | grep observations

# Check inference latency
# Should be <15ms for 50 Hz operation
# Monitor via: ros2 topic hz /bcr_arm/waypoints
```

---

## Troubleshooting

### Problem: "Cannot connect to robot controller"

**Solution**:
1. Verify Ethernet cable is connected
2. Check IP address:
   ```bash
   ip addr show
   ping <robot_controller_ip>
   ```
3. Update interface name in config (`eth0` → actual interface)
4. Check firewall:
   ```bash
   sudo ufw allow from any to any port 5600:5700
   ```

### Problem: "Joint encoder values are NaN"

**Solution**:
1. Verify encoder connections
2. Check sensor configuration in hardware
3. Run encoder test:
   ```bash
   ros2 run ai_control_bridge_hardware test_encoders
   ```

### Problem: "Constraint violations detected"

**Solution**:
1. Reduce trajectory speed in config:
   ```yaml
   control:
     trajectory:
       duration: 3.0  # Increase from 2.0
   ```
2. Lower velocity limits per joint
3. Check model output scaling - may be too aggressive

### Problem: "Inference timeout occurring"

**Solution**:
1. Increase max_time threshold:
   ```yaml
   inference:
     model:
       max_time: 25.0  # Increase from 15.0
   ```
2. Check CPU load: `top`
3. Reduce inference frequency if needed

### Problem: "Control loop missing deadlines"

**Solution**:
1. Enable CPU affinity:
   ```bash
   taskset -c 0-3 ros2 launch ...  # Pin to CPUs 0-3
   ```
2. Reduce other system load
3. Update to RT kernel if not already using it

---

## Safety Guidelines

### Critical Rules

🔴 **NEVER**:
- Disable safety constraints during operation
- Run without emergency stop button armed
- Move robot faster than validated in tests
- Ignore constraint violation warnings
- Bypass watchdog timeout protection

### Operational Checklist

**Before Every Deployment**:
- [ ] Workspace is clear of obstacles
- [ ] Emergency stop button is accessible
- [ ] Team members are briefed
- [ ] All safety interlocks are armed
- [ ] Diagnostic system shows all OK
- [ ] Calibration offsets are recent (<1 week)

**During Operation**:
- [ ] Monitor diagnostics continuously
- [ ] Watch for constraint violations
- [ ] Keep team member with emergency stop
- [ ] Stop if unexpected behavior occurs
- [ ] Log all incidents

**After Operation**:
- [ ] Return robot to home position
- [ ] Document any issues
- [ ] Check for damage/wear
- [ ] Update calibration if needed

---

## Next Steps

1. **Run integration tests** with actual neural network policy
2. **Collect baseline performance data** for future comparison
3. **Document your hardware setup** for reproducibility
4. **Train maintenance plan** for sensors and actuators
5. **Schedule re-calibration** every 30 days or after maintenance

---

## Support & Documentation

- **Framework Repo**: `/home/shivam-maurya/Desktop/GSoC`
- **ROS 2 Docs**: https://docs.ros.org/en/jazzy/
- **ros2_control**: https://github.com/ros-controls/ros2_control
- **Troubleshooting Logs**: `~/.ros/log/`

---

**Deployment Guide Version**: 1.0  
**Last Updated**: March 31, 2026  
**Maintainer**: GSoC Contributors

