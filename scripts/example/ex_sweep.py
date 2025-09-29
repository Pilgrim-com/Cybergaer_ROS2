#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from cybergear_interfaces.srv import SetParam
from std_msgs.msg import Header
import math
import numpy as np

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')
        self.cli = self.create_client(SetParam, 'setparam')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("setparam service not available after 5s")
            rclpy.shutdown()
            return        
        
        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)
        self.timer_period = 0.01/2.0  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Motor Control Group Publisher started")
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.previous_time = self.start_time
        
        self.send_request(control_mode=4, motor_id=0)
        

    def send_request(self,
                     control_mode: int,
                     motor_id: int,
                     communication_type: int = 0,
                     param_name: str = '',
                     param_value: float = 0.0):
        """
        Build and send a SetParam request.
        """
        # 3. Populate the request
        req = SetParam.Request()
        req.control_mode = control_mode
        req.motor_id = motor_id
        req.communication_type = communication_type
        req.param_name = param_name
        req.param_value = param_value

        # 4. Call the service
        self._future = self.cli.call_async(req)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        t = current_time - self.previous_time
        

        max_peak = 2.7    # Example value
        min_peak = 0.2    # Example value
        freq = 1.0/2.5        # Frequency 

        # Calculate midpoint and amplitude
        midpoint = (max_peak + min_peak) / 2
        amplitude = (max_peak - min_peak) / 2

        # Position and velocity commands
        position_cmd = midpoint + amplitude * math.sin(2 * math.pi * freq * t)
        velocity_cmd = amplitude * 2 * math.pi * freq * math.cos(2 * math.pi * freq * t)

        msg = MotorControlGroup()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" 
        
        motor_ids = [1, 5]
        motors = {mid: MotorControl() for mid in motor_ids}
        
        motors[1].set_point.position = np.deg2rad(60.0)
        # motors[2].set_point.position = np.deg2rad(-120.0)
        motors[5].set_point.position = np.deg2rad(60.0)
        # motors[6].set_point.position = np.deg2rad(-120.0)
        
        for mid, mc in motors.items():
            mc.motor_id = mid
            mc.control_mode = 1
            # mc.set_point.position = 0.0
            mc.set_point.velocity = 1.2 if mid in  (7, 8) else 0.0
            mc.set_point.effort   = 10.0 if mid in (7, 8) else 0.0
            mc.set_point.kp       = 50.5 if mid in (7, 8) else 0.0
            mc.set_point.kd       = 0.6  if mid in (7, 8) else 0.0

        # Assign to message and publish
        msg.motor_controls = list(motors.values())
        self.get_logger().info(f"Published commands at t={1/t:.2f}Hz")


        # motor1 = MotorControl()
        # motor1.motor_id = 2
        # motor1.control_mode = 1  
        # motor1.set_point.position = -120.0
        # motor1.set_point.velocity = 1.2
        # motor1.set_point.effort = 10.0
        # motor1.set_point.kp = 50.5
        # motor1.set_point.kd = 0.6

        # motor2 = MotorControl()
        # motor2.motor_id = 2
        # motor2.control_mode = 0
        # motor2.set_point.position = 0.0
        # motor2.set_point.velocity = -1.2
        # motor2.set_point.effort = 10.0
        # motor2.set_point.kp = 50.5
        # motor2.set_point.kd = 0.6

        # motor3 = MotorControl()
        # motor3.motor_id = 3
        # motor3.control_mode = 0
        # motor3.set_point.position = position_cmd
        # motor3.set_point.velocity = velocity_cmd
        # motor3.set_point.effort = 0.0
        # motor3.set_point.kp = 0.0
        # motor3.set_point.kd = 0.0

        # msg.motor_controls = [motor1]  # , motor3]

        self.publisher_.publish(msg)
        self.previous_time = current_time
        # self.get_logger().info(f"Published sweep command at t={t:.2f}s: pos={position_cmd:.2f}, vel={velocity_cmd:.2f}")

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
