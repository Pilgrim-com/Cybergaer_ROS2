#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time
import matplotlib.pyplot as plt
import numpy as np

from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSHistoryPolicy, QoSDurabilityPolicy
)

class JointStateEvaluator(Node):
    def __init__(self,
                 topic_name: str = '/joint_states',
                 settle_time: float = 2.0,
                 record_duration: float = 5.0,
                 cycle_count: int = 10,
                 commanded_angles: dict = None):
        """
        commanded_angles: dict mapping joint name to its reference angle (rad)
        e.g. {'motor_1':1.3978, 'motor_2':-2.4591, ...}
        """
        super().__init__('joint_state_evaluator')
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(
            JointState,
            topic_name,
            self.cb_joint,
            qos
        )

        # parameters
        self.settle_time = settle_time
        self.record_duration = record_duration
        self.cycle_count = cycle_count
        self.cmd = commanded_angles or {}
        
        # internal storage
        self._lock = threading.Lock()
        self._recording = False
        self.times = []           # common timebase for all cycles concatenated
        self.data = {name: [] for name in self.cmd}  # measured angles
        
        # start the experiment thread
        threading.Thread(target=self.run_experiment, daemon=True).start()

    def cb_joint(self, msg: JointState):
        if not self._recording:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self.times.append(t)
            for name, angle in zip(msg.name, msg.position):
                if name in self.data:
                    self.data[name].append(angle)

    def run_experiment(self):
        # wait a bit for everything to come up
        time.sleep(1.0)
        for i in range(1, self.cycle_count+1):
            self.get_logger().info(f'Cycle {i}: settling for {self.settle_time}s…')
            time.sleep(self.settle_time)

            self.get_logger().info(f'Cycle {i}: recording for {self.record_duration}s…')
            with self._lock:
                self.times.clear()
                for name in self.data:
                    self.data[name].clear()
                self._recording = True

            start = time.time()
            while time.time() - start < self.record_duration:
                time.sleep(0.005)

            with self._lock:
                self._recording = False

        self.get_logger().info('All cycles done — processing data and plotting.')
        self.process_and_plot()
        rclpy.shutdown()

    def process_and_plot(self):
        # align time so it starts at zero
        t0 = self.times[0]
        t = np.array(self.times) - t0

        # Log header for benchmark values
        self.get_logger().info("=" * 50)
        self.get_logger().info("MOTOR PERFORMANCE BENCHMARK RESULTS")
        self.get_logger().info("=" * 50)
        
        # conversion factor from radians to degrees
        rad2deg = 180.0 / np.pi
        
        # determine subplot layout based on number of motors
        num_motors = len(self.data)
        rows = int(np.ceil(num_motors / 2))  # 2 columns
        cols = min(2, num_motors)  # at most 2 columns
        
        # Create two figures - one for radians and one for degrees
        fig_rad = plt.figure(figsize=(12, 4*rows))
        fig_deg = plt.figure(figsize=(12, 4*rows))
        
        # add a title to each figure
        fig_rad.suptitle('Joint Angles (radians)', fontsize=16)
        fig_deg.suptitle('Joint Angles (degrees)', fontsize=16)
        
        # for each motor, compute error metrics and plot
        for i, (name, meas) in enumerate(self.data.items(), 1):
            meas = np.array(meas)
            cmd = self.cmd[name]
            err = np.abs(meas - cmd)

            mae = np.mean(err)
            sigma = np.std(err, ddof=1)
            emax = np.max(err)
            
            # Log benchmark values for this motor in radians
            self.get_logger().info(f"Motor: {name}")
            self.get_logger().info(f"  Commanded angle: {cmd:.6f} rad ({cmd * rad2deg:.2f}°)")
            self.get_logger().info(f"  Mean absolute error: {mae:.6f} rad ({mae * rad2deg:.2f}°)")
            self.get_logger().info(f"  Standard deviation: {sigma:.6f} rad ({sigma * rad2deg:.2f}°)")
            self.get_logger().info(f"  Maximum error: {emax:.6f} rad ({emax * rad2deg:.2f}°)")
            
            # Calculate additional metrics
            rms_error = np.sqrt(np.mean(np.square(err)))
            mean_measured = np.mean(meas)
            self.get_logger().info(f"  RMS error: {rms_error:.6f} rad ({rms_error * rad2deg:.2f}°)")
            self.get_logger().info(f"  Mean measured angle: {mean_measured:.6f} rad ({mean_measured * rad2deg:.2f}°)")
            self.get_logger().info(f"  Samples collected: {len(meas)}")
            self.get_logger().info("-" * 50)
            
            # Plot in radians
            ax_rad = fig_rad.add_subplot(rows, cols, i)
            ax_rad.plot(t, meas, label='Measured')
            ax_rad.hlines(cmd, t[0], t[-1], 'r', linestyles='--', label='Commanded')
            ax_rad.set_title(f'{name}: MAE={mae:.4f} rad, σ={sigma:.4f} rad, max={emax:.4f} rad')
            ax_rad.set_xlabel('Time (s)')
            ax_rad.set_ylabel('Angle (rad)')
            ax_rad.legend()
            ax_rad.grid(True, linestyle='--', alpha=0.7)
            
            # Plot in degrees
            ax_deg = fig_deg.add_subplot(rows, cols, i)
            ax_deg.plot(t, meas * rad2deg, label='Measured')
            ax_deg.hlines(cmd * rad2deg, t[0], t[-1], 'r', linestyles='--', label='Commanded')
            # Convert the metrics to degrees
            mae_deg = mae * rad2deg
            sigma_deg = sigma * rad2deg
            emax_deg = emax * rad2deg
            ax_deg.set_title(f'{name}: MAE={mae_deg:.2f}°, σ={sigma_deg:.2f}°, max={emax_deg:.2f}°')
            ax_deg.set_xlabel('Time (s)')
            ax_deg.set_ylabel('Angle (deg)')
            ax_deg.legend()
            ax_deg.grid(True, linestyle='--', alpha=0.7)
        
        # Log summary
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Total motors evaluated: {num_motors}")
        self.get_logger().info(f"Total recording time: {self.record_duration * self.cycle_count:.1f}s ({self.cycle_count} cycles)")
        self.get_logger().info("=" * 50)
        
        # Adjust layout and show both figures
        fig_rad.tight_layout(rect=[0, 0, 1, 0.95])  # make room for suptitle
        fig_deg.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show()

def main(args=None):
    # fill in your SolidWorks reference angles here:
    commanded = {
        'motor_1':  1.3977974058,
        'motor_2': -2.4591098413,
        'motor_5':  1.3870593764,
        'motor_6': -2.4644788560,
    }
    rclpy.init(args=args)
    evaluator = JointStateEvaluator(
        topic_name='/joint_states',
        settle_time=2.0,
        record_duration=5.0,
        cycle_count=10,
        commanded_angles=commanded
    )
    rclpy.spin(evaluator)

if __name__ == '__main__':
    main()
