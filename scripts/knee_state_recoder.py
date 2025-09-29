#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time
import numpy as np
import matplotlib.pyplot as plt

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class KneeTorqueTester(Node):
    def __init__(self,
                 topic_name: str = '/joint_states',
                 motor_name: str = 'motor_2',
                 q2_deg: float = 60.0,
                 g: float = 9.80665,
                 m1: float = 2.347,
                 m2: float = 0.169,
                 m3: float = 0.066,
                 l1: float = 0.20,
                 l2: float = 0.16,
                 l4: float = 0.08,
                 settle_time: float = 2.0,
                 record_duration: float = 5.0,
                 cycle_count: int = 10):
        """
        Node to test static torque of the knee motor (motor_2) for a single knee posture.
        """
        super().__init__('knee_torque_tester')
        self.motor_name = motor_name
        self.q2_deg = q2_deg
        self.g = g
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.l1 = l1
        self.l2 = l2
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
        """
        Compute analytic knee motor torque τ_m2 using:
          θ = 90° − (180° + q2)
          r1 = l1 − l2|cos q2|
          r2 = l1 − l4|cos q2|
          τ_m2 = m1 g cosθ r1 + m2 g cosθ r2 + m3 g cosθ (l1/2)
        """
        q2 = np.deg2rad(self.q2_deg)
        theta = np.deg2rad(90.0 - (180.0 + self.q2_deg))
        term = np.cos(theta)
        r1 = self.l1 - self.l2 * abs(np.cos(q2))
        r2 = self.l1 - self.l4 * abs(np.cos(q2))
        tau = (
            self.m1 * self.g * term * r1 +
            self.m2 * self.g * term * r2 +
            self.m3 * self.g * term * (self.l1 / 2.0)
        )
        return tau

    def run_experiment(self):
        # allow time for subscription
        time.sleep(1.0)

        tau_calc = self.compute_tau_calc()
        self.get_logger().info(f'Knee posture: q2={self.q2_deg}°')
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
        plt.title(f'Knee Motor Torque: q2={self.q2_deg}°')
        plt.xlabel('Sample Index')
        plt.ylabel('Torque (N·m)')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()
        plt.show()

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tester = KneeTorqueTester(
        motor_name='motor_2',
        q2_deg=-60.0,       # select knee posture here
        m1=2.347,
        m2=0.169,
        m3=0.066,
        l1=0.20,
        l2=0.16,
        l4=0.08,
        settle_time=2.0,
        record_duration=5.0,
        cycle_count=10
    )
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
