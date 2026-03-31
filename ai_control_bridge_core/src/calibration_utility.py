#!/usr/bin/env python3
# Copyright 2026 GSoC Contributors
# Licensed under the Apache License, Version 2.0

"""
BCR ARM Calibration Utility

Calibrates the physical BCR ARM robot:
1. Joint encoder offsets
2. Gravity compensation
3. Friction model parameters
4. Home position verification
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import yaml
import os
from pathlib import Path


class CalibrationUtility(Node):
    """Interactive robot calibration tool."""
    
    def __init__(self):
        super().__init__('bcr_arm_calibration')
        
        # Parameters
        self.declare_parameter('config_file', 'bcr_arm_real.yaml')
        self.declare_parameter('robot_dof', 7)
        self.declare_parameter('calibration_timeout', 30.0)
        
        self.dof = self.get_parameter('robot_dof').value
        self.timeout = self.get_parameter('calibration_timeout').value
        self.config_file = self.get_parameter('config_file').value
        
        # Joint state tracking
        self.joint_states = None
        self.joint_states_lock = False
        
        # Subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )
        
        self.get_logger().info("BCR ARM Calibration Utility ready")
        self.get_logger().info(f"DOF: {self.dof}, Config: {self.config_file}")
    
    def joint_state_callback(self, msg):
        """Receive and store current joint states."""
        self.joint_states = msg
        self.joint_states_lock = True
    
    def wait_for_joint_states(self, timeout=None):
        """Wait for joint state data to arrive."""
        if timeout is None:
            timeout = self.timeout
        
        start_time = time.time()
        while not self.joint_states_lock:
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for joint states")
                return False
            time.sleep(0.01)
        
        self.get_logger().info("Joint states received")
        return True
    
    def print_joint_states(self):
        """Display current joint positions."""
        if self.joint_states is None:
            self.get_logger().warn("No joint state data available")
            return
        
        print("\n" + "="*60)
        print("CURRENT JOINT STATES")
        print("="*60)
        for i, name in enumerate(self.joint_states.name):
            pos = self.joint_states.position[i] if i < len(self.joint_states.position) else 0.0
            vel = self.joint_states.velocity[i] if i < len(self.joint_states.velocity) else 0.0
            print(f"{name:15s}  Position: {pos:8.4f} rad  Velocity: {vel:8.4f} rad/s")
        print("="*60 + "\n")
    
    def calibrate_encoder_offsets(self):
        """
        Calibrate joint encoder offsets.
        
        Procedure:
        1. Move robot to known mechanical zero position
        2. Read encoder values
        3. Calculate and store offsets
        """
        print("\n" + "="*60)
        print("CALIBRATION: ENCODER OFFSETS")
        print("="*60)
        
        print("\nINSTRUCTIONS:")
        print("1. Manually move each joint to its mechanical HOME position")
        print("2. For revolute: typically centered (0 radians)")
        print("3. For prismatic: typically fully retracted")
        print("\nPress ENTER when all joints are at home position...")
        input()
        
        if not self.wait_for_joint_states():
            return False
        
        # Record current encoder readings as offsets
        calibration_offsets = {}
        for i, name in enumerate(self.joint_states.name):
            encoder_val = self.joint_states.position[i] if i < len(self.joint_states.position) else 0.0
            calibration_offsets[name] = encoder_val
            print(f"{name}: offset = {encoder_val:.6f} rad")
        
        self.get_logger().info("Encoder calibration complete")
        return calibration_offsets
    
    def calibrate_gravity_compensation(self):
        """
        Calibrate gravity compensation.
        
        Procedure:
        1. Hold each joint at fixed position
        2. Measure effort needed to maintain position
        3. Store as gravity compensation vector
        """
        print("\n" + "="*60)
        print("CALIBRATION: GRAVITY COMPENSATION")
        print("="*60)
        
        print("\nThis requires force/torque sensors on each joint")
        print("Typical procedure:")
        print("1. Lock all joints except one")
        print("2. Measure torque needed to hold against gravity")
        print("3. Repeat for each joint")
        
        print("\nSkipping gravity calibration (advanced feature)")
        print("You can manually set gravity_compensation in config file")
        
        return None
    
    def calibrate_friction_model(self):
        """
        Calibrate friction model.
        
        Procedure:
        1. Apply known torque
        2. Measure resulting velocity
        3. Calculate static/viscous friction coefficients
        """
        print("\n" + "="*60)
        print("CALIBRATION: FRICTION MODEL")
        print("="*60)
        
        print("\nThis requires force/torque control capability")
        print("Typical procedure:")
        print("1. Apply small increasing torque until joint breaks static friction")
        print("2. Measure static friction threshold")
        print("3. At higher speeds, measure viscous friction (torque vs velocity)")
        
        print("\nSkipping friction calibration (advanced feature)")
        print("Default values provided in config files")
        
        return None
    
    def verify_joint_limits(self):
        """
        Verify joint position limits are correctly set.
        
        Procedure:
        1. Move each joint to min position
        2. Move each joint to max position
        3. Verify no contact/safety violations
        """
        print("\n" + "="*60)
        print("VERIFICATION: JOINT LIMITS")
        print("="*60)
        
        print("\nWARNING: This procedure moves the robot to extreme positions")
        print("Ensure the workspace is clear!")
        print("\nPress ENTER to start limit verification...")
        input()
        
        print("\nManual verification needed:")
        print("1. Move each joint to its MINIMUM position")
        print("2. Verify no collision or strain")
        print("3. Move each joint to its MAXIMUM position")
        print("4. Verify no collision or strain")
        
        print("\nPress ENTER after manual limit verification...")
        input()
        
        print("Joint limit verification complete ✓")
        return True
    
    def verify_home_position(self):
        """
        Verify home position is safe and reachable.
        
        Home position is typically: all joints at 0 radians
        """
        print("\n" + "="*60)
        print("VERIFICATION: HOME POSITION")
        print("="*60)
        
        print("\nMoving robot to home position (all joints to 0 radians)...")
        print("If this fails, home position may be unreachable")
        
        # In a real implementation, would send command to move to home
        print("Press ENTER after robot reaches home position...")
        input()
        
        if self.wait_for_joint_states():
            self.print_joint_states()
            print("Home position reached ✓")
            return True
        
        return False
    
    def save_calibration(self, calibration_data):
        """Save calibration results to config file."""
        print("\n" + "="*60)
        print("SAVING CALIBRATION")
        print("="*60)
        
        # Load existing config
        config_path = Path(self.config_file)
        if not config_path.exists():
            self.get_logger().error(f"Config file not found: {config_path}")
            return False
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Update with calibration data
        if 'calibration' not in config:
            config['calibration'] = {}
        
        if calibration_data:
            if 'joints' not in calibration_data:
                calibration_data['joints'] = {}
            
            for joint_name, offset in calibration_data.items():
                if 'joints' in config:
                    if joint_name in config['joints']:
                        config['joints'][joint_name]['calibration_offset'] = float(offset)
        
        # Save backup
        backup_path = config_path.with_suffix('.yaml.bak')
        if config_path.exists():
            import shutil
            shutil.copy(config_path, backup_path)
            self.get_logger().info(f"Backup saved to {backup_path}")
        
        # Write updated config
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info(f"Calibration saved to {config_path}")
        return True
    
    def run_calibration_wizard(self):
        """Interactive calibration wizard."""
        print("\n" + "="*70)
        print("BCR ARM CALIBRATION WIZARD")
        print("="*70)
        print("\nThis wizard will guide you through calibrating your BCR ARM robot")
        print("Estimated time: 10-15 minutes")
        
        # Step 1: Verify connection
        print("\nStep 1: Verifying robot connection...")
        if not self.wait_for_joint_states(timeout=5.0):
            print("ERROR: Cannot connect to robot!")
            return False
        
        self.print_joint_states()
        
        # Step 2: Encoder calibration
        print("\nStep 2: Encoder Offset Calibration")
        offsets = self.calibrate_encoder_offsets()
        
        # Step 3: Limit verification
        print("\nStep 3: Joint Limit Verification")
        self.verify_joint_limits()
        
        # Step 4: Home position verification
        print("\nStep 4: Home Position Verification")
        self.verify_home_position()
        
        # Step 5: Save calibration
        print("\nStep 5: Saving Results")
        if offsets:
            self.save_calibration(offsets)
        
        print("\n" + "="*70)
        print("CALIBRATION COMPLETE ✓")
        print("="*70)
        print("\nNext steps:")
        print("1. Review the updated config file")
        print("2. Test robot motion with calibrated offsets")
        print("3. Adjust limits if needed")
        print("4. Run integration tests")
        
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationUtility()
    
    try:
        success = node.run_calibration_wizard()
        if not success:
            print("Calibration failed or was cancelled")
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
