#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from cybergear_interfaces.srv import SetParam, SetMotionGain
from std_msgs.msg import Header
import math
import numpy as np
import time

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')
        
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep the most recent message
            durability=QoSDurabilityPolicy.VOLATILE
        )        
        
        # Initialize the service availability flag
        self.service_available = False
        
        # Create client for setparam service
        self.cli = self.create_client(SetParam, 'setparam')
        
        # Check if service is available, but don't shutdown if it's not
        if self.cli.wait_for_service(timeout_sec=10.0):
            self.service_available = True
            self.get_logger().info("setparam service is available")
            
            # Send the initial request if service is available
            self.get_logger().info("Sending initial motor enable command...")
            # result = self.send_request_sync(control_mode=4, motor_id=0)
            # if result:
            #     self.get_logger().info("Motors enabled successfully")
            # else:
            #     self.get_logger().error("Failed to enable motors")
        else:
            self.get_logger().warn("setparam service not available after 10s. Continuing without initial setup.")
        
        # Initialize publisher for motor commands
        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', best_effort_qos)
        
        # Create the SetMotionGain service
        self.service = self.create_service(
            SetMotionGain, 
            'set_motion_gain', 
            self.set_motion_gain_callback
        )
        
        # Dictionary to store current motor parameters
        self.motor_params = {}
        
        # Define which motor IDs are valid in your system
        self.valid_motor_ids = [1, 2, 5, 6]
        
        # Default values for each parameter
        self.default_values = {
            'position': 0.0,
            'velocity': 0.0,
            'effort': 0.0,
            'kp': 8.25,
            'kd': 0.75
        }
        
        # Initialize motor params with default values
        for mid in self.valid_motor_ids:
            self.motor_params[mid] = self.default_values.copy()
            
            # Set default positions based on your original code
            if mid == 1 or mid == 5:
                self.motor_params[mid]['position'] = np.deg2rad(40.0)
                # self.motor_params[mid]['kp'] = 5.0
            elif mid == 2 or mid == 6:
                self.motor_params[mid]['position'] = np.deg2rad(-100.0)
        
        self.timer_period = 1.0/500.0
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
        if not self.service_available:
            self.get_logger().warn("setparam service not available, skipping request")
            return False
            
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
        if not self.service_available:
            self.get_logger().warn("setparam service not available, skipping request")
            return
            
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

    def set_motion_gain_callback(self, request, response):
        """Callback function for the SetMotionGain service."""
        self.get_logger().info(f"Received request: motor_id={request.motor_id}, param={request.param_name}")
        
        # Validate motor_id
        if request.motor_id != 0 and request.motor_id not in self.valid_motor_ids:
            response.success = False
            response.message = f"Invalid motor_id: {request.motor_id}. Valid IDs are: {self.valid_motor_ids} or 0 for all motors."
            return response
        
        # Validate param_name
        if request.param_name not in self.default_values:
            response.success = False
            response.message = f"Invalid param_name: {request.param_name}. Valid parameters are: {list(self.default_values.keys())}"
            return response
        
        # Determine which motors to update
        motors_to_update = self.valid_motor_ids if request.motor_id == 0 else [request.motor_id]
        
        # Update the specified parameter for all selected motors
        for mid in motors_to_update:
            # Get current parameters for this motor
            motor_param = self.motor_params.get(mid, self.default_values.copy())
            
            # Update the specified parameter
            if request.param_name == "position":
                motor_param["position"] = np.deg2rad(request.position)
            elif request.param_name == "velocity":
                motor_param["velocity"] = request.velocity
            elif request.param_name == "effort":
                motor_param["effort"] = request.effort
            elif request.param_name == "kp":
                motor_param["kp"] = request.kp
            elif request.param_name == "kd":
                motor_param["kd"] = request.kd
            
            # Store updated parameters
            self.motor_params[mid] = motor_param
        
        # Set response
        response.success = True
        if request.motor_id == 0:
            response.message = f"Updated {request.param_name} for all motors"
        else:
            response.message = f"Updated {request.param_name} for motor {request.motor_id}"
        
        return response

    def timer_callback(self):
        """Regularly publish the current motor parameters to the motor_group_command topic."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        t = current_time - self.previous_time
        
        msg = MotorControlGroup()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        motor_controls = []
        
        for mid in self.valid_motor_ids:
            if mid in self.motor_params:
                mc = MotorControl()
                mc.motor_id = mid
                mc.control_mode = 0  # Position control mode
                mc.set_point.position = self.motor_params[mid]["position"]
                mc.set_point.velocity = self.motor_params[mid]["velocity"]
                mc.set_point.effort = self.motor_params[mid]["effort"]
                mc.set_point.kp = self.motor_params[mid]["kp"]
                mc.set_point.kd = self.motor_params[mid]["kd"]
                
                motor_controls.append(mc)
        
        msg.motor_controls = motor_controls
        self.publisher_.publish(msg)
        self.previous_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorControlGroupPublisher()
        node.get_logger().info("Motor Control Group Publisher running. Use Ctrl+C to stop.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        # Clean shutdown
        try:
            if 'node' in locals():
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Exception during shutdown: {e}")

if __name__ == '__main__':
    main()