#!/usr/bin/env python3
# Copyright 2026 GSoC Contributors
# Licensed under the Apache License, Version 2.0

"""
AI Control Bridge Diagnostics Node

Monitors:
- Control loop frequency and latency
- Constraint violations
- Inference pipeline timing
- Joint state health
- Safety alerts

Publishes diagnostics to /diagnostics topic for aggregation into rqt_runtime_monitor.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
from collections import deque


class DiagnosticsNode(Node):
    """Real-time monitoring of AI control bridge system health."""
    
    def __init__(self):
        super().__init__('ai_control_bridge_diagnostics')
        
        # Parameters
        self.declare_parameter('robot_dof', 7)
        self.declare_parameter('control_frequency', 1000.0)
        self.declare_parameter('inference_frequency', 50.0)
        self.declare_parameter('diagnostics_publish_rate', 10.0)  # Hz
        self.declare_parameter('history_size', 1000)  # Samples for rolling average
        
        self.dof = self.get_parameter('robot_dof').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.inference_freq = self.get_parameter('inference_frequency').value
        self.publish_rate = self.get_parameter('diagnostics_publish_rate').value
        self.history_size = self.get_parameter('history_size').value
        
        # Metrics tracking
        self.control_loop_times = deque(maxlen=self.history_size)
        self.inference_times = deque(maxlen=self.history_size)
        self.constraint_violations = 0
        self.missed_deadlines = 0
        self.last_joint_state_time = None
        self.last_waypoint_time = None
        
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
        
        self.diagnostics_sub = self.create_subscription(
            Float64MultiArray,
            '/control_diagnostics',
            self.control_diagnostics_callback,
            qos
        )
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            qos
        )
        
        # Timer for publishing diagnostics
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_diagnostics
        )
        
        self.get_logger().info("AI Control Bridge Diagnostics Node started")
    
    def joint_state_callback(self, msg):
        """Monitor joint state updates."""
        now = time.time()
        
        if self.last_joint_state_time is not None:
            dt = (now - self.last_joint_state_time) * 1000  # Convert to ms
            self.control_loop_times.append(dt)
            
            # Check for missed deadline (1 ms for 1kHz)
            if dt > 2.0:  # Allow 2x margin
                self.missed_deadlines += 1
        
        self.last_joint_state_time = now
    
    def control_diagnostics_callback(self, msg):
        """Receive diagnostics from control loop."""
        if len(msg.data) >= 3:
            loop_time_us = msg.data[0]  # microseconds
            violations = int(msg.data[1])
            inference_time_us = msg.data[2]  # microseconds
            
            self.control_loop_times.append(loop_time_us / 1000.0)  # Convert to ms
            self.constraint_violations += violations
            self.inference_times.append(inference_time_us / 1000.0)
    
    def publish_diagnostics(self):
        """Publish system health diagnostics."""
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        
        # Control Loop Health
        control_status = DiagnosticStatus()
        control_status.name = "Control Loop"
        control_status.hardware_id = "bcr_arm_controller"
        
        if len(self.control_loop_times) > 0:
            avg_time = sum(self.control_loop_times) / len(self.control_loop_times)
            max_time = max(self.control_loop_times)
            min_time = min(self.control_loop_times)
            
            control_status.values.append(self.make_kv("Average Cycle Time (ms)", f"{avg_time:.3f}"))
            control_status.values.append(self.make_kv("Max Cycle Time (ms)", f"{max_time:.3f}"))
            control_status.values.append(self.make_kv("Min Cycle Time (ms)", f"{min_time:.3f}"))
            control_status.values.append(self.make_kv("Frequency (Hz)", f"{1000.0 / avg_time:.1f}"))
            control_status.values.append(self.make_kv("Samples", str(len(self.control_loop_times))))
            
            # Status determination
            if avg_time < 1.5:  # Should be < 1.0 ms for 1 kHz
                control_status.level = DiagnosticStatus.OK
                control_status.message = "Control loop healthy"
            elif avg_time < 2.0:
                control_status.level = DiagnosticStatus.WARN
                control_status.message = "Control loop slower than expected"
            else:
                control_status.level = DiagnosticStatus.ERROR
                control_status.message = "Control loop exceeding deadline"
        else:
            control_status.level = DiagnosticStatus.ERROR
            control_status.message = "No control loop data"
        
        diagnostics.status.append(control_status)
        
        # Inference Health
        inference_status = DiagnosticStatus()
        inference_status.name = "Inference Processor"
        inference_status.hardware_id = "ai_inference_processor"
        
        if len(self.inference_times) > 0:
            avg_inf_time = sum(self.inference_times) / len(self.inference_times)
            max_inf_time = max(self.inference_times)
            
            inference_status.values.append(self.make_kv("Average Inference Time (ms)", f"{avg_inf_time:.3f}"))
            inference_status.values.append(self.make_kv("Max Inference Time (ms)", f"{max_inf_time:.3f}"))
            inference_status.values.append(self.make_kv("Expected Time (ms)", f"{1000.0 / self.inference_freq:.1f}"))
            
            # 50 Hz -> 20 ms per cycle max
            if avg_inf_time < 15.0:
                inference_status.level = DiagnosticStatus.OK
                inference_status.message = "Inference within budget"
            elif avg_inf_time < 18.0:
                inference_status.level = DiagnosticStatus.WARN
                inference_status.message = "Inference approaching deadline"
            else:
                inference_status.level = DiagnosticStatus.ERROR
                inference_status.message = "Inference exceeding deadline"
        else:
            inference_status.level = DiagnosticStatus.ERROR
            inference_status.message = "No inference data"
        
        diagnostics.status.append(inference_status)
        
        # Safety Health
        safety_status = DiagnosticStatus()
        safety_status.name = "Safety Constraints"
        safety_status.hardware_id = "safety_filter"
        safety_status.values.append(self.make_kv("Total Violations", str(self.constraint_violations)))
        safety_status.values.append(self.make_kv("Missed Deadlines", str(self.missed_deadlines)))
        
        if self.constraint_violations == 0 and self.missed_deadlines == 0:
            safety_status.level = DiagnosticStatus.OK
            safety_status.message = "All constraints satisfied"
        elif self.constraint_violations < 10:
            safety_status.level = DiagnosticStatus.WARN
            safety_status.message = f"{self.constraint_violations} constraint violations detected"
        else:
            safety_status.level = DiagnosticStatus.ERROR
            safety_status.message = f"Multiple constraint violations ({self.constraint_violations})"
        
        diagnostics.status.append(safety_status)
        
        # Publish
        self.diagnostics_pub.publish(diagnostics)
    
    def make_kv(self, key, value):
        """Create a key-value diagnostic pair."""
        from diagnostic_msgs.msg import KeyValue
        kv = KeyValue()
        kv.key = key
        kv.value = value
        return kv


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
