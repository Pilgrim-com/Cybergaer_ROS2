#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from std_msgs.msg import Header
import math

# Import global config access functions
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from config import load_config, get_motor_ids

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')

        # Load configuration to populate global motor IDs
        config_file = self.declare_parameter('config_file', 'config.yaml').value
        load_config(config_file)
        motor_ids = get_motor_ids()
        self.get_logger().info(f"Config loaded. Using motor IDs: {motor_ids}")

        # Use motor IDs from config (defaults to motor 1 and 4 if available)
        self.motor_id_1 = motor_ids[0] if len(motor_ids) > 0 else 1
        self.motor_id_2 = motor_ids[1] if len(motor_ids) > 1 else 4

        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)
        self.timer_period = 0.01/2.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Motor Control Group Publisher started")
        self.start_time = self.get_clock().now().nanoseconds * 1e-9


    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        t = current_time - self.start_time

        amplitude = 1.0  
        frequency = 1.0 
        position_cmd = amplitude * math.sin(2 * math.pi * frequency * t)
        velocity_cmd = amplitude * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)

        msg = MotorControlGroup()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" 

        motor1 = MotorControl()
        motor1.motor_id = self.motor_id_1

        motor1.control_mode = 0
        motor1.set_point.position = position_cmd
        motor1.set_point.velocity = velocity_cmd
        motor1.set_point.effort = 0.0
        motor1.set_point.kp = 0.05
        motor1.set_point.kd = 0.15

        motor2 = MotorControl()
        motor2.motor_id = self.motor_id_2
        motor2.control_mode = 0
        motor2.set_point.position = position_cmd
        motor2.set_point.velocity = velocity_cmd
        motor2.set_point.effort = 0.0
        motor2.set_point.kp = 0.05
        motor2.set_point.kd = 0.15

        # motor3 = MotorControl()
        # motor3.motor_id = 3
        # motor3.control_mode = 0
        # motor3.set_point.position = position_cmd
        # motor3.set_point.velocity = velocity_cmd
        # motor3.set_point.effort = 0.0
        # motor3.set_point.kp = 0.0
        # motor3.set_point.kd = 0.0

        msg.motor_controls = [motor1, motor2]  # , motor3]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published sinewave command at t={t:.2f}s: pos={position_cmd:.2f}, vel={velocity_cmd:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlGroupPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
