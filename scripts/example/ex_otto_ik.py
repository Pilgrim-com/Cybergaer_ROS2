#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from std_msgs.msg import Header

from turtlesim.srv import Spawn

import numpy as np
import math
from typing import List, Tuple

a1, a2 = 0.2, 0.16

class MotorControlGroupPublisher(Node):
    def __init__(self):
        super().__init__('motor_control_group_publisher')
        self.publisher_ = self.create_publisher(MotorControlGroup, 'motor_group_command', 10)
        self.timer_period = 0.01/2.0  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.create_service(Spawn, 'SetPosition', self.handle_ik)
        self.get_logger().info("Motor Control Group Publisher started")
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        
        self.qm = [0.0, 0.0, 0.0, 0.0]
        self.mode = 5 #Disable

    def handle_ik(self, request:Spawn, response:Spawn):
        
        # === Usage ===
        x_target = request.x  
        y_target = request.y

        # x_target, y_target = 0.08113343983870615,-0.2671605231551199
        # 1. Compute elbow-down IK (radians)
        try:
            th1, th2 = get_elbow_down_solution(x_target, y_target)
            self.get_logger().info(f"Joint angles: θ1 = {np.rad2deg(th1):.2f}°, θ2 = {np.rad2deg(th2):.2f}°")
            # 2. Map to motor angles (degrees)
            qm1, qm2 = joint_to_qm(th1, th2)
            self.get_logger().info(f"Motor commands: q_m1 = {np.rad2deg(qm1):.2f}°, q_m2 = {np.rad2deg(qm2):.2f}°")
        except ValueError as e:
            self.get_logger().error(f"IK error: {e}")
            response.name = "Set Position failed"
            return response
        
        self.qm = [qm1, qm2, qm1, qm2]
        
        self.mode = int(request.theta)
        

        
        response.name = "Set Position completed"
        return response

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        t = current_time - self.start_time

        msg = MotorControlGroup()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" 
        
        motor1 = MotorControl()
        motor1.motor_id = 1
        motor1.control_mode = self.mode
        motor1.set_point.position = self.qm[0]
        motor1.set_point.velocity = 1.2
        motor1.set_point.effort = 5.0
        motor1.set_point.kp = 5.5
        motor1.set_point.kd = 0.6
        
        motor2 = MotorControl()
        motor2.motor_id = 2
        motor2.control_mode = self.mode
        motor2.set_point.position = self.qm[1]
        motor2.set_point.velocity = 1.2
        motor2.set_point.effort = 5.0
        motor2.set_point.kp = 5.5
        motor2.set_point.kd = 0.6

        motor5 = MotorControl()
        motor5.motor_id = 5
        motor5.control_mode = self.mode
        motor5.set_point.position = self.qm[2]
        motor5.set_point.velocity = 1.2
        motor5.set_point.effort = 5.0
        motor5.set_point.kp = 5.5
        motor5.set_point.kd = 0.6

        motor6 = MotorControl()
        motor6.motor_id = 6
        motor6.control_mode = self.mode
        motor6.set_point.position = self.qm[3]
        motor6.set_point.velocity = 1.2
        motor6.set_point.effort = 5.0
        motor6.set_point.kp = 5.5
        motor6.set_point.kd = 0.6

        msg.motor_controls = [motor1, motor2, motor5, motor6] 

        self.publisher_.publish(msg)

        # self.mode = 1
        

        # self.get_logger().info(f"Published sweep command at t={t:.2f}s: pos={position_cmd:.2f}, vel={velocity_cmd:.2f}")


def inverse_kinematics(x: float, y: float) -> List[Tuple[float, float]]:
    
    # x, y = -0.2963454975097506,0.09329551487562476
    
    # 2. Analytic IK
    r2 = x**2 + y**2
    cos_q2 = (r2 - a1**2 - a2**2) / (2*a1*a2)
    # Clamp for numerical safety
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)

    solutions = []
    for sign in [+1, -1]:
        q2 = sign * np.arccos(cos_q2)
        phi = np.arctan2(y, x)
        psi = np.arctan2(a2*np.sin(q2), a1 + a2*np.cos(q2))
        q1 = phi - psi
        solutions.append([q1, q2])

    solutions = np.array(solutions)
    return solutions

def get_elbow_down_solution(x: float, y: float) -> Tuple[float, float]:
    """
    Return the elbow-down IK solution for (x, y).
    Negates theta1 to match your sign convention, retains theta2 as-is.
    """
    sols = inverse_kinematics(x, y)
    # pick the first solution where theta2 < 0 (elbow-down)
    sol_neg = next(((th1, th2) for th1, th2 in sols if th2 < 0), None)
    if sol_neg is None:
        raise ValueError(f"No elbow-down solution (θ2<0) for point ({x:.2f}, {y:.2f}).")
    th1, th2 = sol_neg
    if th1 > 0:
        th1 -= 2 * np.pi
    return th1, th2

def joint_to_qm(theta1: float, theta2: float) -> Tuple[float, float]:
    """
    Convert elbow-down joint angles (radians) to motor commands (degrees).
    """
    q1_deg = np.rad2deg(theta1)
    q2_deg = np.rad2deg(theta2)

    # from your config relations:
    qm2 = -q2_deg - 135.59
    qm1 = q1_deg + q2_deg + 100.0

    if qm1 > 0 or qm2 > 0:
        raise ValueError(f"q_m1={qm1:.2f}°, q_m2={qm2:.2f}° exceed zero.")
    return np.deg2rad(qm1), np.deg2rad(qm2)

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
