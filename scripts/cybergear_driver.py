#!/usr/bin/env python3

# Performance benchmarks with different motor counts:
# 1 motor -> 620 hz -> 392 hz
# 2 motor -> 415 hz -> 232 hz
# 3 motor -> 314 hz -> 167 hz
# 4 motor -> 245 hz -> 147 hz
# 5 motor -> 203 hz -> 132 hz
# 6 motor -> 185 hz -> 117 hz

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from cybergear_interfaces.msg import MotorControlGroup, MotorControl
from cybergear_interfaces.srv import SetParam

import time
import array
import functools
import can
from cybergear import CyberGear

# Import from config module with global parameter access
from config import (
    load_config, 
    get_motor_ids, 
    get_motor_count,
    print_motor_summary
)
from controller import CyberGearController
from controller import get_feedback
from controller import MODE_NAMES

class CyberGearROS2Node(Node):
    def __init__(self):
        super().__init__('cybergear_driver_node')
        
        # Load configuration (this populates global variables in config.py)
        config_file = self.declare_parameter('config_file', 'config.yaml').value
        config = load_config(config_file)
        self.get_logger().info("Config loaded")
        
        # Print configuration summary for verification
        print_motor_summary()
        
        # Initialize controller
        self.controller = CyberGearController(config, echo=False)
        self.controller.setup_motors()
        
        # Use global motor IDs instead of extracting from config
        self._motor_ids = get_motor_ids()
        self._num_motors = get_motor_count()
        self._motor_names = [f"motor_{m_id}" for m_id in self._motor_ids]
        self._motor_id_to_index = {m_id: i for i, m_id in enumerate(self._motor_ids)}
        
        # Log motor information using global parameters
        self.get_logger().info(f"Initialized with {self._num_motors} motors: {self._motor_ids}")
        
        # Pre-allocate arrays for joint states (more efficient than appending)
        self._js_names = tuple(self._motor_names)  # Tuple is immutable, slightly faster
        
        # Create services
        self.create_service(SetParam, 'setparam', self.handle_setparam)
        self.create_service(Trigger, 'calibrate', self.handle_calibration)
        
        # Control mode
        self.mode = self.declare_parameter('control_mode', 'velocity').value
        
        # Publishers and subscribers with QoS profiles
        
        # Create Best Effort QoS profile for joint states publisher
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep the most recent message
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Use Best Effort for joint states to minimize latency
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile=best_effort_qos)
        
        # Keep reliable QoS for commands
        self.command_sub = self.create_subscription(
            MotorControlGroup,
            'motor_group_command',
            self.command_callback,
            best_effort_qos
        )
        
        # Timer for regular updates
        self.timer = self.create_timer(1/300.0, self.timer_callback)  # 300 Hz
        
        # State tracking
        self.group_command = None
        self._last_cmd_time = None
        self._log_counter = 0
        
        # Constants
        self._CMD_TIMEOUT = 0.5
        self._LOG_INTERVAL = 50
        
        # Perform initial calibration
        self.controller.calibrate_all_motors()
        
        # Pre-create the JointState message template
        self._js_msg = JointState()
        self._js_msg.name = self._js_names
        self._js_msg.position = [0.0] * self._num_motors
        self._js_msg.velocity = [0.0] * self._num_motors
        self._js_msg.effort = [0.0] * self._num_motors
        
        self._recovery_counter = 0
        self._last_successful_communication = None
        self._last_known_positions = [0.0] * self._num_motors
        self._last_known_velocities = [0.0] * self._num_motors
        self._last_known_efforts = [0.0] * self._num_motors
        
    # Use a dispatch table for control modes to avoid repeated conditionals
    def _handle_control_mode(self, cm, motor_id, request, response):
        """Handle run mode 0-3 (Operation, Position, Speed, Current)"""
        self.controller.cg.set_item_value(motor_id, "run_mode", cm, echo=False)
        response.success = True
        response.message = f"Set control mode to {MODE_NAMES[cm]} for motor {motor_id}"
        return response
        
    def _handle_enable_disable(self, cm, motor_id, request, response):
        """Handle mode 4-5 (Enable/Disable)"""
        enable = (cm == 4)
        msg_action = "Enable" if enable else "Disable"
        
        if motor_id == 0:
            # All motors - use global motor IDs
            for m_id in self._motor_ids:
                self.controller.enable_motor(m_id, cmd=enable)
            response.message = f"{msg_action} all motors"
        else:
            self.controller.enable_motor(motor_id, cmd=enable)
            response.message = f"{msg_action} motor {motor_id}"
            
        response.success = True
        return response
        
    def _handle_param_setting(self, cm, motor_id, request, response):
        """Handle mode 6 (Set arbitrary parameter)"""
        # Dispatch table for communication types → setter functions
        dispatch = {
            18: self.controller.cg.set_item_value,
            8:  self.controller.cg.set_param_value_type8
        }
        
        setter = dispatch.get(request.communication_type)
        if setter:
            if motor_id == 0:
                # All motors - use global motor IDs
                for m_id in self._motor_ids:
                    setter(m_id, request.param_name, request.param_value, echo=False)
                response.message = f"Set {request.param_name} to {request.param_value} for all motors"
                response.success = True
            else:
                setter(motor_id, request.param_name, request.param_value, echo=False)
                response.success = True
                response.message = f"Set {request.param_name} to {request.param_value} for motor {motor_id}"
        else:
            response.success = False
            response.message = "Unsupported communication type"
            
        return response
    
    def handle_setparam(self, request: SetParam, response: SetParam):
        """Service callback for 'setparam'. Optimized via dispatch tables."""
        cm = request.control_mode
        motor_id = request.motor_id
        
        # Using a dispatch table for more efficient control mode handling
        handlers = {
            0: self._handle_control_mode,
            1: self._handle_control_mode,
            2: self._handle_control_mode,
            3: self._handle_control_mode,
            4: self._handle_enable_disable,
            5: self._handle_enable_disable,
            6: self._handle_param_setting
        }
        
        handler = handlers.get(cm)
        if handler:
            response = handler(cm, motor_id, request, response)
        else:
            response.success = False
            response.message = f"Invalid control mode: {cm}"
        
        # Only log successful operations to reduce overhead
        if response.success:
            self.get_logger().info(response.message)
        return response
        
    def handle_calibration(self, request: Trigger, response: Trigger):
        """Service callback for calibrate - calls calibrate_all_motors directly"""
        self.controller.calibrate_all_motors()
        
        response.success = True
        response.message = "Calibration completed for all motors"
        self.get_logger().info(response.message)
        return response
    
    def command_callback(self, msg: MotorControlGroup):
        """Process incoming motor control group commands"""
        # Early return for invalid messages
        if len(msg.motor_controls) != self._num_motors:
            self.get_logger().warn(
                f"Received group command length ({len(msg.motor_controls)}) "
                f"does not match number of motors ({self._num_motors})"
            )
            return
            
        if self.controller.is_any_motor_enabled() == False:
            for m_id in self._motor_ids:
                self.controller.enable_motor(m_id, cmd=True)
        
        # Use a more efficient way to create the command dictionary
        self.group_command = {
            self._motor_ids[i]: msg.motor_controls[i] 
            for i in range(self._num_motors)
        }
        self._last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
        self.controller.send_group_command(self.group_command)
    
    def timer_callback(self):
        """Optimized timer callback for processing commands and publishing joint states with recovery logic"""
        # Get current time only once
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        
        # Command timeout check - more efficient conditional
        if self._last_cmd_time is not None:
            if now_sec - self._last_cmd_time > self._CMD_TIMEOUT:
                self.group_command = None
                for m_id in self._motor_ids:
                    self.controller.enable_motor(m_id, cmd=False)

        # Get joint states and check communication health
        joint_states = self.controller.get_joint_states()
        health_status = self.check_communication_health()
        
        # Periodic motor state logging with StringBuilder pattern to avoid string concatenation
        self._log_counter += 1
        if self._log_counter >= self._LOG_INTERVAL:
            self._log_counter = 0
            # Clear the lru_cache by calling the cache_clear method
            self._get_cached_motor_states.cache_clear()
            
            motor_states = self.controller.get_motor_states()
            log_parts = ["Motor states: "]
            
            for motor_id, state in motor_states.items():
                current_state = state.get('cur_state', 'unknown')
                is_calibrated = state.get('is_calibrated', False)
                calibration_status = "✓" if is_calibrated else "✗"
                log_parts.append(f"Motor {motor_id}: {current_state} ({calibration_status}) | ")
            
            # Add communication health info
            log_parts.append(f"Health: {health_status['responding_motors']}/{health_status['total_motors']} responding | ")
            
            if not (self.group_command is not None and self.controller.is_any_motor_enabled()):
                log_parts.append("No group command received or no motors enabled | ")
            
            # Add recovery status if any motors are being recovered
            if hasattr(self, '_recovery_counter') and self._recovery_counter > 0:
                log_parts.append(f"Recovery attempts: {self._recovery_counter} | ")
            
            self.get_logger().info("".join(log_parts))
        
        # Handle communication issues
        if not health_status["is_healthy"]:
            self._recovery_counter = getattr(self, '_recovery_counter', 0) + 1
            
            if health_status["responding_motors"] == 0:
                # Complete communication failure
                self.get_logger().warn(f"Complete communication failure - attempt {self._recovery_counter}")
                
                # Don't publish anything during complete failure
                if self._recovery_counter <= 5:
                    # Quick recovery attempts
                    self.diagnose_and_recover()
                elif self._recovery_counter % 10 == 0:
                    # Full reset every 10 attempts
                    self.get_logger().error("Performing full system reset")
                    self._perform_full_reset()
                
                return  # Don't publish during complete failure
                
            else:
                # Partial communication failure
                self.get_logger().warn(f"Partial communication failure: {health_status['failed_motors']}")
                
                # Try to recover failed motors
                if self._recovery_counter % 3 == 0:  # Every 3 attempts
                    for motor_id in health_status["failed_motors"]:
                        self.recover_single_motor(motor_id)
        else:
            # Reset recovery counter on healthy communication
            self._recovery_counter = 0

        # Only publish if we have some valid joint states
        if health_status["responding_motors"] > 0:
            # Reuse preallocated JointState message
            self._js_msg.header.stamp = now.to_msg()
            
            # Update arrays with available data
            for i, motor_id in enumerate(self._motor_ids):
                state = joint_states.get(motor_id)
                if state:
                    self._js_msg.position[i] = state.get("position", 0.0)
                    self._js_msg.velocity[i] = state.get("velocity", 0.0) 
                    self._js_msg.effort[i] = state.get("current", 0.0)
                    
                    # Update last known values
                    self._last_known_positions[i] = state.get("position", 0.0)
                    self._last_known_velocities[i] = state.get("velocity", 0.0)
                    self._last_known_efforts[i] = state.get("current", 0.0)
                else:
                    # Use last known values for non-responding motors
                    self._js_msg.position[i] = self._last_known_positions[i]
                    self._js_msg.velocity[i] = 0.0  # Set velocity to 0 for non-responding motors
                    self._js_msg.effort[i] = 0.0    # Set effort to 0 for non-responding motors
            
            # Publish joint states
            self.joint_state_pub.publish(self._js_msg)
    
    def check_communication_health(self):
        """Check overall communication health and return metrics."""
        joint_states = self.controller.get_joint_states()
        
        total_motors = self._num_motors  # Use cached value
        responding_motors = sum(1 for state in joint_states.values() if state is not None)
        
        health_ratio = responding_motors / total_motors if total_motors > 0 else 0
        
        return {
            "total_motors": total_motors,
            "responding_motors": responding_motors,
            "health_ratio": health_ratio,
            "is_healthy": health_ratio >= 0.8,  # Consider healthy if 80%+ motors respond
            "failed_motors": [motor_id for motor_id, state in joint_states.items() if state is None]
        }

    def diagnose_and_recover(self):
        """Comprehensive diagnosis and recovery for motor communication issues."""
        self.get_logger().info("Starting diagnosis and recovery...")
        
        # Try to flush CAN buffer
        try:
            self.controller.cg.rxflush()
        except Exception as e:
            self.get_logger().warn(f"CAN flush failed: {e}")
        
        # Try to re-enable all motors
        for m_id in self._motor_ids:
            try:
                self.controller.enable_motor(m_id, cmd=False)
                time.sleep(0.01)
                self.controller.enable_motor(m_id, cmd=True)
            except Exception as e:
                self.get_logger().warn(f"Failed to re-enable motor {m_id}: {e}")

    def recover_single_motor(self, motor_id):
        """Attempt to recover communication with a single motor."""
        self.get_logger().info(f"Attempting to recover motor {motor_id}")
        
        try:
            # Disable then enable motor
            self.controller.enable_motor(motor_id, cmd=False)
            time.sleep(0.01)
            self.controller.enable_motor(motor_id, cmd=True)
            
            # Test communication
            device_id = self.controller.cg.type0(motor_id, echo=False)
            if device_id is not None:
                self.get_logger().info(f"Motor {motor_id}: Recovery successful")
                return True
                
        except Exception as e:
            self.get_logger().warn(f"Motor {motor_id}: Recovery failed: {e}")
        
        return False

    def _perform_full_reset(self):
        """Perform a complete system reset"""
        try:
            self.get_logger().info("Starting full system reset...")
            
            # Shutdown current CAN connection
            self.controller.cg.shutdown()
            time.sleep(0.1)
            
            # Reinitialize CAN bus
            can_config = self.controller.config.get("can", {})
            self.controller.bus = can.Bus(
                interface=can_config.get("interface", "canalystii"),
                channel=can_config.get("channel", 0),
                bitrate=can_config.get("bitrate", 1000000)
            )
            self.controller.cg = CyberGear(self.controller.bus)
            
            # Setup and calibrate motors
            self.controller.setup_motors()
            self.controller.calibrate_all_motors()
            
            self.get_logger().info("Full system reset completed")
            
        except Exception as e:
            self.get_logger().error(f"Full reset failed: {e}")
    
    # Add caching to reduce repeated calls for motor states
    @functools.lru_cache(maxsize=1)
    def _get_cached_motor_states(self):
        return self.controller.get_motor_states()
        
    @property
    def _cached_motor_states(self):
        return self._get_cached_motor_states()
    
    def destroy_node(self):
        """Clean shutdown procedure"""
        # Disable all motors at once
        for m_id in self._motor_ids:
            self.controller.enable_motor(m_id, cmd=False)
        self.controller.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CyberGearROS2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
        # Efficiently disable all motors
        for m_id in node._motor_ids:
            node.controller.enable_motor(m_id, cmd=False)
        node.controller.shutdown()
        node.get_logger().info("Shutting down controller")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()