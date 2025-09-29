#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from cybergear_interfaces.srv import SetParam
from std_msgs.msg import Header
import math
import numpy as np
import time

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')
        self.cli = self.create_client(SetParam, 'setparam')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("setparam service not available after 5s")
            rclpy.shutdown()
            return        
        
        # Send the initial request and wait for it to complete before continuing
        self.get_logger().info("Sending initial motor enable command...")
        result = self.send_request_sync(control_mode=4, motor_id=0)
        if result:
            self.get_logger().info("Motors enabled successfully")
        else:
            self.get_logger().error("Failed to enable motors")
        
        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)
        self.timer_period = 0.01/2.0  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Motor Control Group Publisher started")
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.previous_time = self.start_time

    def send_request_sync(self,
                     control_mode: int,
                     motor_id: int,
                     communication_type: int = 0,
                     param_name: str = '',
                     param_value: float = 0.0,
                     timeout_sec: float = 5.0) -> bool:
        """
        Build and send a SetParam request synchronously, waiting for the response.
        Returns True if successful, False if timed out or error occurred.
        """
        # Build the request
        req = SetParam.Request()
        req.control_mode = control_mode
        req.motor_id = motor_id
        req.communication_type = communication_type
        req.param_name = param_name
        req.param_value = param_value

        # Call the service asynchronously
        future = self.cli.call_async(req)
        
        # Wait for the response with timeout
        start_time = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"Service call timed out after {timeout_sec} seconds")
                return False
                
        # Check if the call was successful
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response.message}")
                return True
            else:
                self.get_logger().error(f"Service call failed: {response.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Service call raised an exception: {str(e)}")
            return False

    def send_request(self,
                     control_mode: int,
                     motor_id: int,
                     communication_type: int = 0,
                     param_name: str = '',
                     param_value: float = 0.0):
        """
        Build and send a SetParam request asynchronously.
        """
        # Build the request
        req = SetParam.Request()
        req.control_mode = control_mode
        req.motor_id = motor_id
        req.communication_type = communication_type
        req.param_name = param_name
        req.param_value = param_value

        # Call the service
        future = self.cli.call_async(req)
        
        # Add a callback to handle the response when it comes
        future.add_done_callback(self.request_callback)

    def request_callback(self, future):
        """
        Callback for handling service response
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response.message}")
            else:
                self.get_logger().error(f"Service call failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call raised an exception: {str(e)}")

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        t = current_time - self.previous_time
        
        max_peak = np.deg2rad(70)    # Example value
        min_peak = np.deg2rad(-30)    # Example value
        freq = 1.0/2.5        # Frequency 

        # Calculate midpoint and amplitude
        midpoint = (max_peak + min_peak) / 2
        amplitude = (max_peak - min_peak) / 2

        # Position and velocity commands
        position_cmd = midpoint + amplitude * math.sin(0.1*2 * math.pi * freq * current_time)
        velocity_cmd = amplitude * 2 * math.pi * freq * 0.1*math.cos(0.1*2 * math.pi * freq * current_time)

        msg = MotorControlGroup()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" 
        
        motor_ids = [1, 2, 5, 6]
        motors = {mid: MotorControl() for mid in motor_ids}
        
        motors[1].set_point.position = position_cmd #np.deg2rad(60.0)
        motors[2].set_point.position = np.deg2rad(-100.0)
        motors[5].set_point.position = position_cmd #np.deg2rad(60.0)
        motors[6].set_point.position = np.deg2rad(-100.0)
        
        for mid, mc in motors.items():
            mc.motor_id = mid
            mc.control_mode = 1
            # mc.set_point.position = 0.0
            mc.set_point.velocity = 0.0 if mid in  (1, 6) else 0.0
            mc.set_point.effort   = 0.0 if mid in (1, 6) else 0.0
            mc.set_point.kp       = 5.0 if mid in (1, 6) else 0.0
            mc.set_point.kd       = 0.5  if mid in (1, 6) else 0.0

        # Assign to message and publish
        msg.motor_controls = list(motors.values())
        self.get_logger().info(f"Published commands {position_cmd } at t={1/t:.2f}Hz")

        self.publisher_.publish(msg)
        self.previous_time = current_time

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