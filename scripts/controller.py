import time
import can
import math
from cybergear import CyberGear, uint_to_float
from struct import pack, unpack, iter_unpack

MODE_NAMES = {
        0: "operation", 1: "position", 2: "speed", 3: "current",
        4: "enable", 5: "disable", 6: "set_param"
    }

def get_feedback(cg, motor_id, echo=False):
    """
    ดึง feedback จากมอเตอร์ผ่านคำสั่ง type2
    แปลงค่าที่อ่านได้ออกเป็น:
      - position: ช่วง -4π ถึง +4π
      - velocity: ช่วง -30 ถึง +30
      - current:  ช่วง -12 ถึง +12
    """
    r = cg.type2(motor_id, echo=echo)
    if r is not None:
        alarm_code = (r[0] >> 16) & 0xff
        fb = r[1]
        position = uint_to_float(fb[0], -4*math.pi, 4*math.pi)
        velocity = uint_to_float(fb[1], -30, 30)
        current  = uint_to_float(fb[2], -12, 12)
        return alarm_code, position, velocity, current
    return None


class CyberGearController:
    """
    Controller สำหรับจัดการมอเตอร์หลายตัว โดยอ่าน configuration จากไฟล์ YAML
    """
    def __init__(self, config: dict, echo=False):
        self.config = config
        self.echo = echo
        
        can_config = config.get("can", {})
        self.bus = can.Bus(interface=can_config.get("interface", "canalystii"),
                           channel=can_config.get("channel", 0),
                           bitrate=can_config.get("bitrate", 1000000))
        self.cg = CyberGear(self.bus)
        
        self.motors = config.get("motors", [])
        self._flip_map = {
            motor["id"]: motor.get("flip", False)
            for motor in self.motors
        }
        
        self._raw_pos_zero_map = {
            motor["id"]: motor.get("raw_pos_zero", 0.0)
            for motor in self.motors
        }
        
        self.control_config = config.get("control", {})
        
        # Initialize individual motor states with expanded state tracking
        self.motor_states = {
            motor["id"]: {
                "cur_state": MODE_NAMES[5],  # Default to "disable"
                "prev_state": MODE_NAMES[5],
                "is_calibrated": False,
                "last_run_mode": None,        # Track last run mode sent
                "last_commands": {},          # Dict to track last command values for each mode
                "last_position": 0.0,         # Last position command
                "last_velocity": 0.0,         # Last velocity command
                "last_effort": 0.0            # Last effort command
            }
            for motor in self.motors
        }

    def command_motor(self, motor_id, target_value, echo=False):
        """
        ส่งคำสั่งมอเตอร์เฉพาะเมื่อ:
          1) รันโหมดเปลี่ยนไป (run_mode)
          2) ค่าเป้าหมายเปลี่ยนไป (position, velocity, current)
        Now uses motor_states for tracking instead of cg cache variables
        """
        mode = target_value.control_mode
        set_point = target_value.set_point
        
        # Get motor state for this motor
        if motor_id not in self.motor_states:
            # If motor not found in states, initialize it
            self.motor_states[motor_id] = {
                "cur_state": MODE_NAMES[5],  # Default to "disable"
                "prev_state": MODE_NAMES[5],
                "is_calibrated": False,
                "last_run_mode": None,
                "last_commands": {},
                "last_position": 0.0,
                "last_velocity": 0.0,
                "last_effort": 0.0
            }
        
        motor_state = self.motor_states[motor_id]

        # --- 1) Ensure run_mode is sent once per change ---
        prev_mode = motor_state["last_run_mode"]
        if prev_mode != mode:
            # map mode integer directly to run_mode field
            self.cg.set_item_value(motor_id, "run_mode", mode, echo=echo)
            motor_state["last_run_mode"] = mode
            motor_state["prev_state"] = motor_state["cur_state"]
            motor_state["cur_state"] = MODE_NAMES[mode]
            
            if echo:
                print(f"[run_mode] Motor {motor_id} switched to mode {mode}")

        # --- 2) For modes 1,2,3: only send if target changed ---
        # if mode in (1, 2, 3):
        #     current_target = None
        #     if mode == 1:
        #         current_target = set_point.position
        #         last_cmd = motor_state.get("last_position")
        #     elif mode == 2:
        #         current_target = set_point.velocity
        #         last_cmd = motor_state.get("last_velocity")
        #     elif mode == 3:
        #         current_target = set_point.effort
        #         last_cmd = motor_state.get("last_effort")
                
        #     if last_cmd == current_target : #and mode == motor_state["last_commands"].get(mode):
        #         # no change → skip
        #         return True

        # --- dispatch to existing command logic ---
        if mode == 0:
            success = self.cg.type1_si(
                motor_id,
                set_point.effort, set_point.position,
                set_point.velocity, set_point.kp, set_point.kd,
                echo=echo
            )
            if success:
                motor_state["last_position"] = set_point.position
                motor_state["last_velocity"] = set_point.velocity
                motor_state["last_effort"] = set_point.effort
                motor_state["cur_state"] = MODE_NAMES[0]  # "operation"
                if echo:
                    print(f"[MIT] Motor {motor_id} commanded: {target_value}")
            return success

        elif mode == 1:
            success = self.cg.set_item_value(motor_id, "loc_ref", set_point.position, echo=echo)
            if success:
                motor_state["last_position"] = set_point.position
                motor_state["cur_state"] = MODE_NAMES[1]  # "position"
                if echo:
                    print(f"[POSITION] Motor {motor_id} → {set_point.position}")
            return success

        elif mode == 2:
            success = self.cg.set_item_value(motor_id, "spd_ref", set_point.velocity, echo=echo)
            if success:
                motor_state["last_velocity"] = set_point.velocity
                motor_state["cur_state"] = MODE_NAMES[2]  # "speed"
                if echo:
                    print(f"[VELOCITY] Motor {motor_id} → {set_point.velocity}")
            return success

        elif mode == 3:
            success = self.cg.set_item_value(motor_id, "iq_ref", set_point.effort, echo=echo)
            if success:
                motor_state["last_effort"] = set_point.effort
                motor_state["cur_state"] = MODE_NAMES[3]  # "current"
                if echo:
                    print(f"[CURRENT] Motor {motor_id} → {set_point.effort}")
            return success

        elif mode == 4:
            success = self.cg.type3(motor_id, echo=echo)
            if success:
                motor_state["prev_state"] = motor_state["cur_state"]
                motor_state["cur_state"] = MODE_NAMES[4]  # "enable"
                if echo:
                    print(f"[ENABLE] Motor {motor_id}")
            return success

        elif mode == 5:
            success = self.cg.type4(motor_id, fault=False, echo=echo)
            if success:
                motor_state["prev_state"] = motor_state["cur_state"]
                motor_state["cur_state"] = MODE_NAMES[5]  # "disable"
                if echo:
                    print(f"[DISABLE] Motor {motor_id}")
            return success

        elif mode == 6:
            success = self.cg.type6(motor_id, echo=echo)
            if success:
                motor_state["prev_state"] = motor_state["cur_state"]
                motor_state["cur_state"] = MODE_NAMES[6]  # "set_param"
                if echo:
                    print(f"[ZERO] Motor {motor_id}")
            return success

        else:
            if echo:
                print("Invalid mode. Valid: 0 (MIT), 1 (pos), 2 (spd), 3 (cur), 4–6 control.")
            return False

    def setup_motors(self):
        for motor in self.motors:
            motor_id = motor.get("id")
            control_mode = motor.get("control_mode", 1)
            limit_spd = motor.get("limit_spd", 30.0)
            limit_cur = motor.get("limit_cur", 23.0)
            mech_offset = motor.get("MechOffset", 0.0)
            
            # self.cg.set_param_value_type8(motor_id, "MechOffset", mech_offset, echo=False)
            # self.cg.set_param_value_type8(motor_id, "MechPos_init", 4.52, echo=False)
            
            self.cg.type4(motor_id, fault=False, echo=self.echo) # Stop Motor
            
            # Update individual motor state
            if motor_id in self.motor_states:
                self.motor_states[motor_id]["prev_state"] = self.motor_states[motor_id]["cur_state"]
                self.motor_states[motor_id]["cur_state"] = MODE_NAMES[5]  # "disable"
            
            self.cg.set_item_value(motor_id, "run_mode", control_mode, echo=self.echo)
            self.cg.set_item_value(motor_id, "limit_spd", limit_spd, echo=self.echo)
            self.cg.set_item_value(motor_id, "limit_cur", limit_cur, echo=self.echo)
            
            self.cg.set_param_value_type8(motor_id, "loc_kp", 1.25, echo=True)
            self.cg.set_param_value_type8(motor_id, "spd_kp",1.0, echo=True)
            self.cg.set_param_value_type8(motor_id, "spd_ki", 0.05, echo=True)               
            
            # self.cg.set_param_value_type8(motor_id, "loc_kp", 5.0, echo=True)
            # self.cg.set_param_value_type8(motor_id, "spd_kp",2.0, echo=True)
            # self.cg.set_param_value_type8(motor_id, "spd_ki", 0*0.021*2, echo=True)            
            
            # Track settings in motor state
            self.motor_states[motor_id]["last_run_mode"] = control_mode
            
            # self.cg.type6(motor_id, echo=self.echo) #Set Zero
            
            # self.cg.type3(motor_id, echo=self.echo) #Enable motor
            print(f"Motor {motor_id} setup completed but not calibrate yet, mode: {control_mode}.")
            
            
    def enable_motor(self, motor_id, cmd: bool):
        # Track previous state before changing
        if motor_id in self.motor_states:
            self.motor_states[motor_id]["prev_state"] = self.motor_states[motor_id]["cur_state"]
        
        if(cmd == True):
            self.cg.type3(motor_id, echo=self.echo) # Enable Motor
            if motor_id in self.motor_states:
                self.motor_states[motor_id]["cur_state"] = MODE_NAMES[4]  # "enable"
        elif(cmd == False):
            self.cg.type4(motor_id, fault=False, echo=self.echo) # Stop Motor
            if motor_id in self.motor_states:
                self.motor_states[motor_id]["cur_state"] = MODE_NAMES[5]  # "disable"
            
    def set_zero_motor(self, motor_id):
        self.cg.type6(motor_id, echo=self.echo)
        # Update state tracking for this motor
        if motor_id in self.motor_states:
            self.motor_states[motor_id]["prev_state"] = self.motor_states[motor_id]["cur_state"]
            self.motor_states[motor_id]["cur_state"] = MODE_NAMES[6]  # "set_param"

    def command_all(self, mode: str, target_value: float):
        for motor in self.motors:
            motor_id = motor.get("id")
            self.command_motor(motor_id, mode, target_value, echo=self.echo)

    def create_item_command_frame(self, motor_id, field_name, target_value, echo=False):
        field = next((f for f in self.cg._fields_ if f["name"] == field_name), None)
        if field is None:
            if echo:
                print(f"Field {field_name} not found!")
            return None
        code = field["code"]
        fmt = field["format"]
        
        if fmt != 'f':
            if echo:
                print("Field format not supported in batch command.")
            return None


        try:
            data = pack("<Hxxf", code, target_value)
        except Exception as e:
            if echo:
                print(f"Packing error for motor {motor_id}: {e}")
            return None

        arbitration_id = (motor_id & 0xff) | (0 << 8) | ((18 & 0x1f) << 24)
        frame = can.Message(arbitration_id=arbitration_id,
                            data=data,
                            is_extended_id=True)
        if echo:
            print(f"Created frame for motor {motor_id}: ID=0x{frame.arbitration_id:08x} Data={frame.data.hex()}")
        return frame

    def create_can_frame(self, motor_id, mode, target_value, master_id=0xFD):
        def float_to_uint(x: float, x_min: float, x_max: float):
            if x > x_max:
                x = x_max
            elif x < x_min:
                x = x_min
            return int((x - x_min) * 65535 / (x_max - x_min))
        
        
        mode_dict = {'position': 1, 'velocity': 2, 'torque': 3}

        cmd = mode_dict.get(mode, 0) 

        if mode == 'position':
            target_int = float_to_uint(target_value, -math.pi, math.pi)
        elif mode == 'velocity':
            target_int = float_to_uint(target_value, -20, 20) 
        elif mode == 'torque':
            target_int = float_to_uint(target_value, -5, 5)   
        else:
            target_int = 0

        data = pack('>Hxxxxxx', target_int) 

        arbitration_id = ((cmd & 0x1F) << 24) | ((master_id & 0xFFFF) << 8) | (motor_id & 0xFF)

        frame = can.Message(arbitration_id=arbitration_id,
                            data=data,
                            is_extended_id=True)

        return frame
                            
    def send_group_command1(self, commands: dict, mode: str):
        field_map = {
            "position": "loc_ref",
            "velocity": "spd_ref",
            "current": "iq_ref"
        }
        field_name = field_map.get(mode.lower())
        if field_name is None:
            print("Invalid mode specified. Use 'position', 'velocity', or 'current'.")
            return

        frames = []
        for motor in self.motors:
            motor_id = motor.get("id")
            target_value = commands.get(motor_id)
            if target_value is None:
                continue
            frame = self.create_item_command_frame(motor_id, field_name, target_value, echo=self.echo)
            if frame is not None:
                frames.append(frame)
        self.cg.send_batch(frames)


    def send_group_command(self, commands: dict):
        """
        Send a command to each motor in 'commands', applying calibration
        and flip corrections as needed. Each motor is treated independently.
        """
        for motor_id, target in commands.items():
            # Check if this specific motor is calibrated
            motor_calibrated = (motor_id in self.motor_states and 
                               self.motor_states[motor_id]["is_calibrated"])
            
            # O(1) lookup for flip
            if self._flip_map.get(motor_id, False):
                sp = target.set_point
                raw_pos_zero = self._raw_pos_zero_map.get(motor_id, 0.0)
                
                if motor_calibrated:
                    sp.position = -(sp.position + raw_pos_zero)
                else:
                    sp.position = -sp.position
                
                sp.velocity = -sp.velocity
                sp.effort = -sp.effort
            else:
                sp = target.set_point
                
                if motor_calibrated:
                    raw_pos_zero = self._raw_pos_zero_map.get(motor_id, 0.0)
                    sp.position = sp.position + raw_pos_zero

            # Single call to command_motor per motor
            self.command_motor(motor_id, target, echo=self.echo)
                
    def get_joint_states(self):
        """
        Get joint states for all motors with per-motor calibration state and improved error handling.
        """
        joint_states = {}
        communication_errors = []
        
        for motor in self.motors:
            motor_id = motor.get("id")
            flip = motor.get("flip", False)
            raw_pos_zero = self._raw_pos_zero_map.get(motor_id, 0.0)
            
            # Check if this specific motor is calibrated
            motor_calibrated = (motor_id in self.motor_states and 
                            self.motor_states[motor_id]["is_calibrated"])
            
            # Get feedback from the motor with retry mechanism
            fb = None
            max_retries = 2
            
            for attempt in range(max_retries):
                try:
                    fb = get_feedback(self.cg, motor_id, echo=self.echo)
                    if fb is not None:
                        break
                    else:
                        # Log communication issue
                        if attempt == 0:  # Only log on first attempt to avoid spam
                            communication_errors.append(f"Motor {motor_id}: No response")
                            
                except Exception as e:
                    communication_errors.append(f"Motor {motor_id}: Exception {e}")
                    
                # Small delay between retries
                if attempt < max_retries - 1:
                    time.sleep(0.001)
            
            if fb is not None:
                try:
                    _, pos, vel, curr = fb
                    
                    if motor_calibrated:
                        # Calibrated state - apply position offset
                        if flip:
                            joint_states[motor_id] = {
                                "position": -pos - raw_pos_zero,
                                "velocity": -vel,
                                "current": -curr
                            }
                        else:
                            joint_states[motor_id] = {
                                "position": pos - raw_pos_zero,
                                "velocity": vel,
                                "current": curr
                            }
                    else:
                        # Uncalibrated state - no position offset
                        if flip:
                            joint_states[motor_id] = {
                                "position": -pos,
                                "velocity": -vel,
                                "current": -curr
                            }
                        else:
                            joint_states[motor_id] = {
                                "position": pos,
                                "velocity": vel,
                                "current": curr
                            }
                except Exception as e:
                    joint_states[motor_id] = None
                    communication_errors.append(f"Motor {motor_id}: Data parsing error {e}")
            else:
                joint_states[motor_id] = None
        
        # Log communication errors if any occurred
        if communication_errors and self.echo:
            print(f"Communication errors: {communication_errors}")
                
        return joint_states
    
    def calibrate_motor(self, motor_id):
        """
        Set a specific motor as calibrated.
        """
        if motor_id in self.motor_states:
            self.motor_states[motor_id]["is_calibrated"] = True
            return True
        return False
    
    def calibrate_all_motors(self):
        """
        Set all motors as calibrated.
        """
        for motor_id in self.motor_states:
            self.motor_states[motor_id]["is_calibrated"] = True
        
    def all_motors_calibrated(self):
        """
        Check if all motors are calibrated.
        """
        return all(state["is_calibrated"] for state in self.motor_states.values())
    
    def is_any_motor_enabled(self):
        """
        Return True if any motor's current state is one of the enabled modes.
        """
        # Define which MODE_NAMES indices count as "enabled"
        enabled_states = {
            MODE_NAMES[0],
            MODE_NAMES[1],
            MODE_NAMES[2],
            MODE_NAMES[4],
        }
        # any() returns True if any motor's cur_state is in enabled_states
        return any(
            state["cur_state"] in enabled_states
            for state in self.motor_states.values()
        )

    
    def get_motor_current_state(self, motor_id):
        """
        Get the current state of a specific motor.
        """
        if motor_id in self.motor_states:
            return self.motor_states[motor_id]["cur_state"]
        return None
    
    def get_motor_states(self):
        """
        Get states for all motors.
        """
        return self.motor_states

    def shutdown(self):
        self.cg.shutdown()
        
    def check_motor_communication(self, motor_id):
        """
        Check if a specific motor is communicating properly.
        Returns True if motor responds, False otherwise.
        """
        try:
            # Try to get device ID
            device_id = self.cg.type0(motor_id, echo=False)
            if device_id is not None:
                return True
                
            # Try to get feedback
            fb = get_feedback(self.cg, motor_id, echo=False)
            return fb is not None
            
        except Exception:
            return False

    def diagnose_communication_issues(self):
        """
        Diagnose communication issues with all motors.
        Returns a dict with motor_id: status
        """
        status = {}
        for motor in self.motors:
            motor_id = motor.get("id")
            
            # Check basic communication
            can_communicate = self.check_motor_communication(motor_id)
            
            # Check motor state
            motor_state = self.motor_states.get(motor_id, {}).get("cur_state", "unknown")
            
            # Check if motor is enabled
            is_enabled = motor_state in ["enable", "operation", "position", "speed"]
            
            status[motor_id] = {
                "can_communicate": can_communicate,
                "current_state": motor_state,
                "is_enabled": is_enabled,
                "is_calibrated": self.motor_states.get(motor_id, {}).get("is_calibrated", False)
            }
            
        return status