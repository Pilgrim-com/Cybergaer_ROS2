#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
import numpy as np
import sys
PATH_TO_PROJECT = "/home/tanawatp/volta_ws/src/volta_controller/Utils"
sys.path.append(PATH_TO_PROJECT)
from KinematicsFunc import SolveInverseKinematics
from UtilsFunc import DefinePlant
from pydrake.math import RigidTransform, RotationMatrix
# from scipy import stats

class CyberGearCalibrationNode(Node):
    def __init__(self):
        super().__init__('cybergear_calibration_node')
        p,i = DefinePlant(model_name="Volta_rev4",
                        is_fixed=True,
                        fixed_frame="torso")
        desire_torso_height = 0.46
        is_success_L,q_L = SolveInverseKinematics(model_series="Volta",
                                    plant=p,
                                    is_left=True,
                                    base_P_e=np.array([0.0, 0.125, -desire_torso_height]),
                                    base_R_e=RotationMatrix.MakeXRotation(0.0))
        is_success_R,q_R = SolveInverseKinematics(model_series="Volta",
                                    plant=p,
                                    is_left=False,
                                    base_P_e=np.array([0.0, -0.125, -desire_torso_height]),
                                    base_R_e=RotationMatrix.MakeXRotation(0.0))
        # q_L = np.zeros(6)
        # q_R = np.zeros(6)
        # Motor configuration from your image
        self.motor_config = {
            'motor_1': {
                'desired_pos_init': q_L[0],
                'flip': True
            },
            'motor_2': {
                'desired_pos_init': q_L[1],
                'flip': True
            },
            'motor_3': {
                'desired_pos_init': q_L[2],
                'flip': False
            },
            'motor_4': {
                'desired_pos_init': q_L[3],
                'flip': False
            },
            'motor_5': {
                'desired_pos_init': q_L[4],
                'flip': False
            },
            'motor_6': {
                'desired_pos_init': q_L[5],
                'flip': False
            },
            'motor_7': {
                'desired_pos_init': q_R[0],
                'flip': True
            },
            'motor_8': {
                'desired_pos_init': q_R[1],
                'flip': True
            },
            'motor_9': {
                'desired_pos_init': q_R[2],
                'flip': False
            },
            'motor_10': {
                'desired_pos_init': q_R[3],
                'flip': False
            },
            'motor_11': {
                'desired_pos_init': q_R[4],
                'flip': False
            },
            'motor_12': {
                'desired_pos_init': q_R[5],
                'flip': False
            }
        }
        
        # Calibration parameters
        self.target_samples = 500
        self.samples_collected = {}
        self.position_samples = {}
        
        # Initialize sample storage
        for motor_name in self.motor_config.keys():
            self.samples_collected[motor_name] = 0
            self.position_samples[motor_name] = []
        
        # Create QoS profile matching the publisher
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile=best_effort_qos
        )
        
        self.get_logger().info(f"Starting calibration - collecting {self.target_samples} samples per motor...")
    def joint_state_callback(self, msg: JointState):
        """Process incoming joint state messages"""
        # Process each motor's position data
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.motor_config:
                if self.samples_collected[joint_name] < self.target_samples:
                    self.position_samples[joint_name].append(msg.position[i])
                    self.samples_collected[joint_name] += 1
        # self.get_logger().info(f"Samples collected: {self.samples_collected}")
        # Check if calibration is complete
        if all(count >= self.target_samples for count in self.samples_collected.values()):
            self.finish_calibration()
    
    def calculate_mode(self, data, num_bins=50):
        """Calculate mode using histogram binning approach for continuous data"""
        # For continuous position data, we use histogram to find the most frequent range
        counts, bin_edges = np.histogram(data, bins=num_bins)
        
        # Find the bin with the highest count
        max_count_index = np.argmax(counts)
        
        # Return the center of the bin with highest frequency
        mode_value = (bin_edges[max_count_index] + bin_edges[max_count_index + 1]) / 2
        
        return mode_value
    
    def finish_calibration(self):
        """Calculate and display calibration results"""
        self.get_logger().info("Calibration Motor")
        
        for motor_name, config in self.motor_config.items():
            # Calculate raw_pos_init using mode instead of mean
            raw_pos_init = self.calculate_mode(self.position_samples[motor_name])
            desired_pos_init = config['desired_pos_init']
            raw_pos_zero = raw_pos_init - desired_pos_init
            
            flip_status = "Flip" if config['flip'] else "No Flip"
            # self.get_logger().info(f"raw_pos_init = raw_pos in Calibration Posture")
            # self.get_logger().info(f"desired_pos_init = desired_pos of raw_pos_init in Calibration Posture")
            self.get_logger().info("")
            self.get_logger().info(f"{motor_name.capitalize()} is {flip_status}")
            # self.get_logger().info("")
            # self.get_logger().info(f"raw_pos_init = {raw_pos_init}")
            # self.get_logger().info(f"desired_pos_init = np.deg2rad({desired_pos_init})")
            # self.get_logger().info(f"raw_pos_zero = raw_pos_init - desired_pos_init")
            self.get_logger().info(f"raw_pos_zero: {raw_pos_zero}")
            # self.get_logger().info("")
            # self.get_logger().info("")
        
        # Shutdown after displaying results
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CyberGearCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Calibration interrupted")
    finally:
        if not rclpy.ok():
            return
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()