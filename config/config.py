#!/usr/bin/env python3
"""
Configuration loader for CyberGear driver with global parameter access.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory

# ============================================================================
# GLOBAL VARIABLES - Populated when load_config() is called
# ============================================================================
MOTOR_IDS = []
MOTOR_CONFIG = {}
CAN_CONFIG = {}
_CONFIG_LOADED = False

# ============================================================================
# MAIN CONFIGURATION LOADER
# ============================================================================
def load_config(config_file):
    """
    Load configuration from YAML file and populate global variables.
    
    Args:
        config_file: Path to config file (absolute or relative to package share)
        
    Returns:
        dict: Complete configuration dictionary
    """
    global MOTOR_IDS, MOTOR_CONFIG, CAN_CONFIG, _CONFIG_LOADED
    
    # Resolve config file path
    if not os.path.isabs(config_file):
        pkg_share = get_package_share_directory('cybergear_driver')
        config_file = os.path.join(pkg_share, 'config', config_file)
    
    # Load YAML
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract and store motor information globally
    motors = config.get('motors', [])
    MOTOR_IDS = [motor['id'] for motor in motors]
    
    # Store full motor config indexed by motor ID for easy lookup
    MOTOR_CONFIG = {motor['id']: motor for motor in motors}
    
    # Store CAN configuration
    CAN_CONFIG = config.get('can', {})
    
    # Mark as loaded
    _CONFIG_LOADED = True
    
    return config

# ============================================================================
# GLOBAL PARAMETER ACCESS FUNCTIONS
# ============================================================================
def get_motor_ids():
    """
    Get list of all motor IDs.
    
    Returns:
        list: Motor IDs [1, 2, 3, ...]
    """
    _check_config_loaded()
    return MOTOR_IDS

def get_motor_count():
    """
    Get total number of motors.
    
    Returns:
        int: Number of motors
    """
    _check_config_loaded()
    return len(MOTOR_IDS)

def get_motor_config(motor_id):
    """
    Get configuration dictionary for a specific motor.
    
    Args:
        motor_id: Motor CAN ID
        
    Returns:
        dict: Motor configuration or None if not found
    """
    _check_config_loaded()
    return MOTOR_CONFIG.get(motor_id)

def get_all_motor_configs():
    """
    Get all motor configurations.
    
    Returns:
        dict: Dictionary mapping motor_id -> config
    """
    _check_config_loaded()
    return MOTOR_CONFIG

def get_can_config():
    """
    Get CAN bus configuration.
    
    Returns:
        dict: CAN configuration (interface, channel, bitrate)
    """
    _check_config_loaded()
    return CAN_CONFIG

def get_motor_param(motor_id, param_name, default=None):
    """
    Get a specific parameter value for a motor.
    
    Args:
        motor_id: Motor CAN ID
        param_name: Parameter name (e.g., 'limit_spd', 'flip', 'raw_pos_zero')
        default: Default value if parameter not found
        
    Returns:
        Parameter value or default
    """
    _check_config_loaded()
    motor_cfg = MOTOR_CONFIG.get(motor_id)
    if motor_cfg:
        return motor_cfg.get(param_name, default)
    return default

def is_motor_flipped(motor_id):
    """
    Check if a motor has its direction flipped.
    
    Args:
        motor_id: Motor CAN ID
        
    Returns:
        bool: True if flipped, False otherwise
    """
    return get_motor_param(motor_id, 'flip', False)

def get_motor_zero_offset(motor_id):
    """
    Get the raw position zero offset for a motor.
    
    Args:
        motor_id: Motor CAN ID
        
    Returns:
        float: Zero position offset in radians
    """
    return get_motor_param(motor_id, 'raw_pos_zero', 0.0)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
def _check_config_loaded():
    """Internal function to ensure config has been loaded."""
    if not _CONFIG_LOADED:
        raise RuntimeError(
            "Configuration not loaded! Call load_config() first before "
            "accessing global parameters."
        )

def print_motor_summary():
    """Print a summary of loaded motor configuration."""
    _check_config_loaded()
    
    print("=" * 60)
    print("CYBERGEAR MOTOR CONFIGURATION SUMMARY")
    print("=" * 60)
    print(f"Total Motors: {get_motor_count()}")
    print(f"Motor IDs: {MOTOR_IDS}")
    print(f"\nCAN Interface: {CAN_CONFIG.get('interface', 'N/A')}")
    print(f"CAN Channel: {CAN_CONFIG.get('channel', 'N/A')}")
    print(f"CAN Bitrate: {CAN_CONFIG.get('bitrate', 'N/A')}")
    print("\nMotor Details:")
    print("-" * 60)
    
    for motor_id in MOTOR_IDS:
        config = MOTOR_CONFIG[motor_id]
        print(f"Motor {motor_id}:")
        print(f"  Control Mode: {config.get('control_mode', 'N/A')}")
        print(f"  Speed Limit: {config.get('limit_spd', 'N/A')} rad/s")
        print(f"  Current Limit: {config.get('limit_cur', 'N/A')} A")
        print(f"  Flipped: {config.get('flip', False)}")
        print(f"  Zero Offset: {config.get('raw_pos_zero', 0.0):.4f} rad")
        if 'mit' in config:
            print(f"  MIT Kp: {config['mit'].get('kp', 'N/A')}")
            print(f"  MIT Kd: {config['mit'].get('kd', 'N/A')}")
        print()
    
    print("=" * 60)

def is_valid_motor_id(motor_id):
    """
    Check if a motor ID is valid (exists in configuration).
    
    Args:
        motor_id: Motor CAN ID to check
        
    Returns:
        bool: True if motor ID exists in config
    """
    _check_config_loaded()
    return motor_id in MOTOR_IDS

# ============================================================================
# EXAMPLE USAGE (for testing)
# ============================================================================
if __name__ == '__main__':
    # Load config
    config = load_config('config.yaml')
    
    # Print summary
    print_motor_summary()
    
    # Access global parameters
    print("\nAccessing global parameters:")
    print(f"Motor IDs: {get_motor_ids()}")
    print(f"Motor Count: {get_motor_count()}")
    print(f"Motor 1 flipped: {is_motor_flipped(1)}")
    print(f"Motor 1 zero offset: {get_motor_zero_offset(1)}")