#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time
import numpy as np
import matplotlib.pyplot as plt

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class HipTorqueTester(Node):
    def __init__(self,
                 topic_name: str = '/joint_states',
                 motor_name: str = 'motor_1',
                 q1_deg: float = 60.0,
                 q2_deg: float = -120.0,
                 g: float = 9.80665,
                 m1: float = 1.2,
                 m2: float = 0.8,
                 l2: float = 0.25,
                 l3: float = 0.05,
                 l4: float = 0.10,
                 settle_time: float = 2.0,
                 record_duration: float = 5.0,
                 cycle_count: int = 10):
        """
        Node to test static torque of the hip motor (motor_1) for a single leg posture.
        """
        super().__init__('hip_torque_tester')
        self.motor_name = motor_name
        self.q1_deg = q1_deg
        self.q2_deg = q2_deg
        self.g = g
        self.m1 = m1
        self.m2 = m2
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.settle_time = settle_time
        self.record_duration = record_duration
        self.cycle_count = cycle_count

        # storage for measured efforts
        self.efforts = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(
            JointState, topic_name, self.joint_callback, qos)

        self._lock = threading.Lock()
        self._recording = False

        threading.Thread(target=self.run_experiment, daemon=True).start()

    def joint_callback(self, msg: JointState):
        """Record motor effort when _recording is True."""
        if not self._recording:
            return
        try:
            idx = msg.name.index(self.motor_name)
            effort = msg.effort[idx]
        except ValueError:
            return
        with self._lock:
            self.efforts.append(effort)

    def compute_tau_calc(self):
        """Compute analytic torque τ_m1 for the given joint angles."""
        q1 = np.deg2rad(self.q1_deg)
        q2 = np.deg2rad(self.q2_deg)
        numerator = (self.m1*self.l2 + self.m2*self.l4) * self.g * np.cos(q2)
        angle_term = np.cos(np.deg2rad(90.0) - abs(q1 + q2))
        tau = numerator / angle_term
        return -tau

    def run_experiment(self):
        # allow time for subscription
        time.sleep(1.0)

        tau_calc = self.compute_tau_calc()
        self.get_logger().info(f'Leg posture: q1={self.q1_deg}°, q2={self.q2_deg}°')
        self.get_logger().info(f'Calculated τ_calc = {tau_calc:.4f} N·m')

        for cycle in range(1, self.cycle_count + 1):
            self.get_logger().info(f'Settle for {self.settle_time}s...')
            time.sleep(self.settle_time)

            with self._lock:
                self.efforts.clear()
                self._recording = True

            self.get_logger().info(f'Recording for {self.record_duration}s...')
            time.sleep(self.record_duration)

            with self._lock:
                self._recording = False

            self.get_logger().info(f'Cycle {cycle} samples: {len(self.efforts)}')

        # analyze results
        efforts = np.array(self.efforts)
        mean_tau = np.mean(efforts)
        std_tau = np.std(efforts, ddof=1)
        max_dev = np.max(np.abs(efforts - tau_calc))
        mte = np.mean(np.abs(efforts - tau_calc))
        cov = std_tau / mean_tau if mean_tau != 0 else float('nan')

        self.get_logger().info('--- Results ---')
        self.get_logger().info(f'Mean measured τ: {mean_tau:.4f} N·m')
        self.get_logger().info(f'Standard deviation: {std_tau:.4f} N·m')
        self.get_logger().info(f'Max deviation: {max_dev:.4f} N·m')
        self.get_logger().info(f'Mean torque error (MTE): {mte:.4f} N·m')
        self.get_logger().info(f'Coefficient of Variation: {cov*100:.2f}%')

        # plot
        plt.figure(figsize=(8,4))
        plt.plot(efforts, label='Measured τ')
        plt.hlines(tau_calc, 0, len(efforts)-1, 'r', linestyles='--', label='τ_calc')
        plt.title(f'Hip Motor Torque: q1={self.q1_deg}°, q2={self.q2_deg}°')
        plt.xlabel('Sample Index')
        plt.ylabel('Torque (N·m)')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()
        plt.show()

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tester = HipTorqueTester(
        motor_name='motor_2',
        q1_deg=60.0,       # select posture here
        q2_deg=-120.0,     # select posture here
        m1=2.347,
        m2=0.066,
        l2=0.16,
        l3=0.08,
        l4=0.16/2.0,
        settle_time=2.0,
        record_duration=5.0,
        cycle_count=10
    )
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
