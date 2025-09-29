#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from std_msgs.msg import Header

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

        # Use first motor ID from config
        self.motor_id = motor_ids[0] if motor_ids else 1

        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Motor Control Group Publisher started")

    def timer_callback(self):
        msg = MotorControlGroup()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"


        motor1 = MotorControl()
        motor1.motor_id = self.motor_id
        motor1.control_mode = 0  # Operation mode
        motor1.set_point.velocity = 0.0
        motor1.set_point.position = 0.0
        motor1.set_point.effort = 0.0
        motor1.set_point.kp = 0.01
        motor1.set_point.kd = 0.01

        msg.motor_controls = [motor1]

        self.get_logger().info("Publishing MotorGroupControl message")
        self.publisher_.publish(msg)

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
