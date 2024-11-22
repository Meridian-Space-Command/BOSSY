"""
Spacecraft Simulator

This module implements a complete spacecraft simulation including all major subsystems:
- OBC (On-Board Computer)
- ADCS (Attitude Determination and Control)
- Power
- Thermal
- Communications
- Payload (Earth Observation Camera)
- Data Storage

The simulator runs at 10Hz and provides CCSDS-compliant telemetry and telecommand handling.
It interfaces with the universe simulator for environmental data and maintains
compatibility with YAMCS for ground communications.
"""

import numpy as np
from enum import Enum
from datetime import datetime, timedelta, timezone
import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import struct
import socket
from threading import Thread, Lock
from time import sleep

from . import spacecraft_config as sc_cfg
from . import universe_config as uni_cfg
from . import universe_sim as universe

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('spacecraft_simulator.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

@dataclass
class SpacecraftState:
    """Complete spacecraft state representation"""
    def __init__(self):
        self.time = datetime.now(timezone.utc)
        self.position = np.array([7000.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 7.5, 0.0])
        self.attitude = np.array([1., 0., 0., 0.])
        self.angular_velocity = np.zeros(3)
        self.power_level = 1.0
        self.temperature = 293.0  # Initialize to 20°C
        self.mode = 'NOMINAL'

class SubsystemBase:
    """Base class for all subsystems"""
    def __init__(self, spacecraft):
        self.spacecraft = spacecraft
        self.telemetry = {}
        self.lock = Lock()

    def update(self) -> None:
        """Update subsystem state"""
        raise NotImplementedError

    def handle_command(self, command: bytes) -> None:
        """Handle ADCS telecommands
        
        Commands:
        0x01: Set target attitude (quaternion) [16 bytes: 4 floats]
        0x02: Set control mode (NADIR/SUNPOINTING) [1 byte]
        0x03: Reset wheels to zero speed [0 bytes]
        """
        cmd_id = command[0]
        payload = command[1:]
        
        try:
            if cmd_id == 0x01:
                # Parse the quaternion values (4 x 32-bit floats)
                self.target_attitude = np.frombuffer(payload, dtype=np.float32)
                # Normalize the input quaternion
                self.target_attitude = self.target_attitude / np.linalg.norm(self.target_attitude)
                self.mode = 'CUSTOM'  # Internal mode only
                logger.info(f"ADCS: New target attitude set: {self.target_attitude}")
            elif cmd_id == 0x02:
                mode = payload[0]
                if mode == 0:
                    self.mode = 'NADIR'
                    pos = self.spacecraft.state.position
                    nadir_direction = -pos / np.linalg.norm(pos)
                    self.target_attitude = self._calculate_nadir_quaternion(nadir_direction)
                elif mode == 1:
                    self.mode = 'SUNPOINTING'
                    self.target_attitude = self._calculate_sun_pointing_quaternion(
                        self.spacecraft.environment.sun_direction
                    )
                logger.info(f"ADCS: Control mode set to {self.mode}")
            elif cmd_id == 0x03:
                self.wheel_speeds = np.zeros(3)
                logger.info("ADCS: Wheel speeds reset to zero")

            self.spacecraft.send_event(
                EventSeverity.INFO,
                "ADCS",
                f"Command {cmd_id:#x} executed successfully"
            )
        except Exception as e:
            self.spacecraft.send_event(
                EventSeverity.ERROR,
                "ADCS",
                f"Command {cmd_id:#x} failed: {str(e)}"
            )

    def get_telemetry(self) -> Dict:
        """Get current telemetry"""
        with self.lock:
            return self.telemetry.copy()
        
class EventSeverity(Enum):
    INFO = 0
    WARNING = 1
    ERROR = 2
    CRITICAL = 3

class ADCS(SubsystemBase):
    """Attitude Determination and Control System"""
    
    def __init__(self, spacecraft):
        super().__init__(spacecraft)
        self.target_attitude = np.array([1., 0., 0., 0.])  # Quaternion
        self.wheel_speeds = np.zeros(3)  # rad/s
        self.magnetorquer_dipoles = np.zeros(3)  # Am²
        self.mode = 'NOMINAL'
        
        # Load configuration
        cfg = sc_cfg.ADCS_CONFIG
        self.max_slew_rate = cfg['max_slew_rate']
        self.wheel_max_speed = cfg['reaction_wheels']['max_speed']
        self.wheel_moment = cfg['reaction_wheels']['moment']
        self.mag_max_dipole = cfg['magnetorquers']['max_dipole']
        self.Kp = 0.005  # Reduce proportional gain further
        self.Kd = 0.2    # Increase derivative gain for better damping
        
        # Create inertia tensor from config
        self.inertia = np.diag([
            cfg['inertia']['Ixx'],
            cfg['inertia']['Iyy'],
            cfg['inertia']['Izz']
        ])
        
        # Add pointing status
        self.pointing_status = 'SLEWING'  # Can be 'SLEWING', 'POINTING', 'UNKNOWN'
        self.pointing_settled_time = 0.0  # Time spent within tolerance
        self.settling_time_required = 10.0  # Increase settling time requirement
        
        # Adjust tolerances for pointing status
        self.pointing_tolerance = sc_cfg.ADCS_CONFIG['pointing_tolerance']
        self.rate_tolerance = sc_cfg.ADCS_CONFIG['rate_tolerance']
        
    def update(self) -> None:
        """Update ADCS state"""
        env = self.spacecraft.environment
        dt = self.spacecraft.dt
        
        # Update target attitude based on mode
        if self.mode == 'NADIR':
            pos = self.spacecraft.state.position
            nadir_direction = -pos / np.linalg.norm(pos)
            self.target_attitude = self._calculate_nadir_quaternion(nadir_direction)
        
        # Normalize quaternions to prevent drift
        current_attitude = self.spacecraft.state.attitude
        current_attitude = current_attitude / np.linalg.norm(current_attitude)
        self.target_attitude = self.target_attitude / np.linalg.norm(self.target_attitude)
        
        # Calculate quaternion error (this is critical for correct control)
        q_error = self._quaternion_multiply(
            self._quaternion_conjugate(current_attitude),
            self.target_attitude
        )
        
        # Convert to axis-angle for control (shorter path rotation)
        angle = 2.0 * np.arccos(np.clip(q_error[0], -1.0, 1.0))
        if angle > 1e-6:
            axis = q_error[1:] / np.sin(angle/2.0)
        else:
            axis = np.zeros(3)
        
        # PD Control law with better scaling
        angular_velocity = self.spacecraft.state.angular_velocity
        
        # Scale control gains by error magnitude
        error_scale = np.clip(angle / np.pi, 0.0, 1.0)  # Normalize to [0,1]
        
        # Proportional term (scaled by angle)
        torque_p = self.Kp * axis * error_scale
        
        # Derivative term (angular velocity damping)
        torque_d = -self.Kd * angular_velocity
        
        # Combine and limit total torque
        control_torque = np.clip(
            torque_p + torque_d,
            -0.001,  # Max torque limit
            0.001
        )
        
        # Apply control torque through actuators
        self._apply_control_torque(control_torque, env)
        
        # Update pointing status with hysteresis
        old_status = self.pointing_status
        
        # Check if within pointing and rate tolerances
        within_pointing = angle <= self.pointing_tolerance
        within_rate = np.linalg.norm(angular_velocity) <= self.rate_tolerance
        
        if within_pointing and within_rate:
            self.pointing_settled_time += dt
            if self.pointing_settled_time >= self.settling_time_required:
                if self.pointing_status != 'POINTING':
                    logger.info(f"ADCS: Achieved {self.mode} pointing")
                self.pointing_status = 'POINTING'
        else:
            self.pointing_settled_time = 0.0
            if self.pointing_status != 'SLEWING':
                logger.info(f"ADCS: Lost pointing, slewing to {self.mode}")
            self.pointing_status = 'SLEWING'
        
        # Generate event if status changed
        if old_status != self.pointing_status:
            severity = EventSeverity.INFO if self.pointing_status == 'POINTING' else EventSeverity.WARNING
            self.spacecraft.send_event(
                severity,
                "ADCS",
                f"Pointing status changed to {self.pointing_status} in {self.mode} mode"
            )
        
        # Update telemetry
        with self.lock:
            self.telemetry.update({
                'mode': self.mode,
                'pointing_status': self.pointing_status,
                'pointing_error': angle,
                'rate_error': np.linalg.norm(angular_velocity),
                'attitude': self.spacecraft.state.attitude.tolist(),
                'angular_velocity': self.spacecraft.state.angular_velocity.tolist(),
                'wheel_speeds': self.wheel_speeds.tolist(),
                'magnetorquer_dipoles': self.magnetorquer_dipoles.tolist(),
                'target_attitude': self.target_attitude.tolist()
            })

        # Log debug info periodically
        if int(self.spacecraft.state.time.timestamp()) % 10 == 0:
            logger.debug(f"ADCS Debug: angle={angle:.3f} rate={np.linalg.norm(angular_velocity):.6f} "
                        f"torque_p={np.linalg.norm(torque_p):.6f} torque_d={np.linalg.norm(torque_d):.6f}")
    
    
    def handle_command(self, command: bytes) -> None:
        """Handle ADCS telecommands
        
        Commands:
        0x01: Set target attitude (quaternion) [16 bytes: 4 doubles]
        0x02: Set control mode (NADIR/SUNPOINTING/CUSTOM) [1 byte]
        0x03: Reset wheels to zero speed [0 bytes]
        0x04: Set wheel speed directly [12 bytes: 3 doubles]
        0x05: Set magnetorquer dipole [12 bytes: 3 doubles]
        """
        cmd_id = command[0]
        payload = command[1:]
        
        if cmd_id == 0x01:
            self.target_attitude = np.frombuffer(payload, dtype=np.float64)
            logger.info(f"ADCS: New target attitude set: {self.target_attitude}")
        elif cmd_id == 0x02:
            mode = payload[0]
            if mode == 0:
                self.mode = 'NADIR'
            elif mode == 1:
                self.mode = 'SUNPOINTING'
            else:
                self.mode = 'CUSTOM'
            logger.info(f"ADCS: Control mode set to {self.mode}")
            
            # Force immediate update of target attitude
            if self.mode == 'NADIR':
                pos = self.spacecraft.state.position
                nadir_direction = -pos / np.linalg.norm(pos)
                self.target_attitude = self._calculate_nadir_quaternion(nadir_direction)
            elif self.mode == 'SUNPOINTING':
                self.target_attitude = self._calculate_sun_pointing_quaternion(
                    self.spacecraft.environment.sun_direction
                )
        elif cmd_id == 0x03:
            self.wheel_speeds = np.zeros(3)
            logger.info("ADCS: Wheel speeds reset to zero")
        elif cmd_id == 0x04:
            self.wheel_speeds = np.frombuffer(payload, dtype=np.float64)
            logger.info(f"ADCS: Wheel speeds set to {self.wheel_speeds}")
        elif cmd_id == 0x05:
            self.magnetorquer_dipoles = np.frombuffer(payload, dtype=np.float64)
            logger.info(f"ADCS: Magnetorquer dipoles set to {self.magnetorquer_dipoles}")

        try:
            self.spacecraft.send_event(
                EventSeverity.INFO,
                "ADCS",
                f"Command {cmd_id:#x} executed successfully"
            )
        except Exception as e:
            self.spacecraft.send_event(
                EventSeverity.ERROR,
                "ADCS",
                f"Command {cmd_id:#x} failed: {str(e)}"
            )

    def _quaternion_error(self, q_current: np.ndarray, q_desired: np.ndarray) -> np.ndarray:
        """Calculate quaternion error"""
        return universe.quaternion_multiply(
            universe.quaternion_conjugate(q_current),
            q_desired
        )

    def _calculate_sun_pointing_quaternion(self, sun_direction: np.ndarray) -> np.ndarray:
        """Calculate quaternion to point solar panels (+Y axis) at sun"""
        # Normalize sun vector
        sun_norm = sun_direction / np.linalg.norm(sun_direction)
        
        # Define spacecraft +Y axis (solar panel normal)
        spacecraft_y = np.array([0.0, 1.0, 0.0])
        
        # Get rotation axis and angle
        rotation_axis = np.cross(spacecraft_y, sun_norm)
        rotation_axis_norm = np.linalg.norm(rotation_axis)
        
        if rotation_axis_norm < 1e-10:
            # Sun vector already aligned with Y-axis
            if np.dot(spacecraft_y, sun_norm) > 0:
                # We're pointing the right way
                return np.array([1.0, 0.0, 0.0, 0.0])
            else:
                # We're pointing 180° wrong way
                return np.array([0.0, 0.0, 0.0, 1.0])  # 180° rotation around Z
            
        rotation_axis = rotation_axis / rotation_axis_norm
        cos_angle = np.dot(spacecraft_y, sun_norm)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        
        # Convert to quaternion
        return np.array([
            np.cos(angle/2),
            rotation_axis[0] * np.sin(angle/2),
            rotation_axis[1] * np.sin(angle/2),
            rotation_axis[2] * np.sin(angle/2)
        ])

    def _calculate_nadir_quaternion(self, nadir_direction: np.ndarray) -> np.ndarray:
        """Calculate quaternion to point payload (-Z axis) at nadir"""
        # Normalize nadir vector
        nadir_norm = nadir_direction / np.linalg.norm(nadir_direction)
        
        # Define spacecraft -Z axis (payload direction)
        spacecraft_z = np.array([0.0, 0.0, -1.0])
        
        # Get rotation axis and angle
        rotation_axis = np.cross(spacecraft_z, nadir_norm)
        rotation_axis_norm = np.linalg.norm(rotation_axis)
        
        if rotation_axis_norm < 1e-10:
            # Nadir vector already aligned with Z-axis
            if np.dot(spacecraft_z, nadir_norm) < 0:
                # We're pointing the right way
                return np.array([1.0, 0.0, 0.0, 0.0])
            else:
                # We're pointing 180° wrong way
                return np.array([0.0, 1.0, 0.0, 0.0])  # 180° rotation around X
        
        rotation_axis = rotation_axis / rotation_axis_norm
        cos_angle = np.dot(spacecraft_z, nadir_norm)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        
        # Convert to quaternion
        return np.array([
            np.cos(angle/2),
            rotation_axis[0] * np.sin(angle/2),
            rotation_axis[1] * np.sin(angle/2),
            rotation_axis[2] * np.sin(angle/2)
        ])

    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def _quaternion_conjugate(self, q: np.ndarray) -> np.ndarray:
        """Return conjugate of quaternion"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def _apply_control_torque(self, torque: np.ndarray, env: dict) -> None:
        """Apply control torque using reaction wheels"""
        dt = self.spacecraft.dt
        
        # Add torque limiting to prevent oscillations
        max_torque = 0.0005  # Reduce maximum torque
        torque = np.clip(torque, -max_torque, max_torque)
        
        # Update reaction wheel speeds
        self.wheel_speeds += torque * dt / self.wheel_moment
        
        # Clip wheel speeds to maximum
        self.wheel_speeds = np.clip(
            self.wheel_speeds,
            -self.wheel_max_speed,
            self.wheel_max_speed
        )
        
        # Update spacecraft angular velocity
        self.spacecraft.state.angular_velocity += torque * dt / np.diag(self.inertia)
        
        # Update attitude quaternion using angular velocity
        omega = self.spacecraft.state.angular_velocity
        omega_norm = np.linalg.norm(omega)
        if omega_norm > 1e-10:
            axis = omega / omega_norm
            angle = omega_norm * dt
            dq = np.array([
                np.cos(angle/2),
                axis[0] * np.sin(angle/2),
                axis[1] * np.sin(angle/2),
                axis[2] * np.sin(angle/2)
            ])
            self.spacecraft.state.attitude = self._quaternion_multiply(
                self.spacecraft.state.attitude, dq
            )
            # Normalize quaternion
            self.spacecraft.state.attitude /= np.linalg.norm(self.spacecraft.state.attitude)

class PowerSystem(SubsystemBase):
    """Power System"""
    
    def __init__(self, spacecraft):
        super().__init__(spacecraft)
        self.battery_charge = sc_cfg.POWER_CONFIG['battery']['initial_charge']
        self.solar_panel_power = 0.0
        
    def update(self) -> None:
        env = self.spacecraft.environment
        
        # Solar panel power generation based on sun angle and eclipse
        panel_area = sc_cfg.POWER_CONFIG['solar_panels']['area_per_panel']
        efficiency = sc_cfg.POWER_CONFIG['solar_panels']['efficiency']
        
        # Calculate sun angle to panels (simplified)
        sun_vector = env.sun_direction
        panel_normal = self.spacecraft.state.attitude[1:4]  # Use spacecraft attitude
        sun_angle = np.arccos(np.dot(sun_vector, panel_normal))
        
        # Calculate power based on angle and eclipse
        if env.is_eclipsed:
            self.solar_panel_power = 0.0
        else:
            self.solar_panel_power = (
                panel_area * efficiency * env.solar_flux * 
                max(0, np.cos(sun_angle))  # Only positive contributions
            )
        
        # Power consumption varies by mode
        base_consumption = sc_cfg.POWER_CONFIG['power_consumption']['base']
        subsystem_consumption = sc_cfg.POWER_CONFIG['power_consumption']
        
        # Add mode-specific consumption
        total_consumption = base_consumption
        if self.spacecraft.adcs.mode != 'SAFE':
            total_consumption += subsystem_consumption['adcs']
        if hasattr(self.spacecraft.payload, 'imaging') and self.spacecraft.payload.imaging:
            total_consumption += subsystem_consumption['payload']
        
        # Battery charge update
        battery_capacity = sc_cfg.POWER_CONFIG['battery']['capacity']
        charge_delta = (self.solar_panel_power - total_consumption) * self.spacecraft.dt / (battery_capacity * 3600)
        self.battery_charge = np.clip(self.battery_charge + charge_delta, 0, 1)
        
        # Update spacecraft state
        self.spacecraft.state.power_level = self.battery_charge
        
        # Update telemetry
        with self.lock:
            self.telemetry.update({
                'battery_charge': self.battery_charge,
                'solar_power': self.solar_panel_power,
                'power_consumption': total_consumption,
                'is_eclipsed': env.is_eclipsed
            })

        if self.battery_charge < 0.1:
            self.spacecraft.send_event(
                EventSeverity.CRITICAL,
                "POWER",
                f"Critical battery level: {self.battery_charge:.2%}"
            )
        
        if self.solar_panel_power < 1.0 and not self.spacecraft.environment.is_eclipsed:
            self.spacecraft.send_event(
                EventSeverity.WARNING,
                "POWER",
                f"Low solar panel power while in sunlight: {self.solar_panel_power:.1f}W"
            )
    
    def handle_command(self, command: bytes) -> None:
        """Handle Power System telecommands
        
        Commands:
        0x01: Set power mode (NORMAL/LOW_POWER/SAFE) [1 byte]
        0x02: Toggle subsystem power [2 bytes: subsystem_id, state]
        0x03: Force battery charge level [8 bytes: double]
        """
        cmd_id = command[0]
        payload = command[1:]
        
        if cmd_id == 0x01:
            mode = payload[0]
            modes = {0: 'NORMAL', 1: 'LOW_POWER', 2: 'SAFE'}
            self.power_mode = modes.get(mode, 'NORMAL')
            logger.info(f"Power: Mode set to {self.power_mode}")
        elif cmd_id == 0x02:
            subsystem_id = payload[0]
            power_state = bool(payload[1])
            self.subsystem_power[subsystem_id] = power_state
            logger.info(f"Power: Subsystem {subsystem_id} power set to {power_state}")
        elif cmd_id == 0x03:
            self.battery_charge = np.frombuffer(payload, dtype=np.float64)[0]
            logger.info(f"Power: Battery charge forced to {self.battery_charge}")


class Communications(SubsystemBase):
    """Communications System"""
    
    def __init__(self, spacecraft):
        super().__init__(spacecraft)
        self.tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tc_socket.bind(('127.0.0.1', 10025))
        self.tc_socket.settimeout(0.1)
        self.packet_sequence_count = 0
        self.epoch = sc_cfg.MISSION_START_TIME
        
    def update(self) -> None:
        """Add this method to check for incoming commands"""
        try:
            data, addr = self.tc_socket.recvfrom(1024)
            logger.info(f"Received telecommand packet: {list(data)}")  # Add this line
            self.spacecraft.handle_telecommand(data)
        except socket.timeout:
            pass  # No command received
    
    def send_telemetry_packet(self, apid: int, data: bytes) -> None:
        """Send CCSDS Space Packet"""
        # Debug the incoming data
        logger.debug(f"First 4 bytes of packet data: {list(data[:4])}")
        
        # Construct CCSDS primary header
        version = 0
        type_flag = 0  # Telemetry
        secondary_flag = 0
        sequence_flags = 3  # Standalone packet
        packet_length = len(data) - 1
        
        primary_header = struct.pack(
            '>HHH',
            (version << 13) | (type_flag << 12) | (secondary_flag << 11) | apid,
            (sequence_flags << 14) | (self.packet_sequence_count & 0x3FFF),
            packet_length
        )
        
        packet = primary_header + data
        self.tm_socket.sendto(packet, ('127.0.0.1', 10015))
        self.packet_sequence_count = (self.packet_sequence_count + 1) & 0x3FFF
    
    def handle_command(self, command: bytes) -> None:
        """Handle Communications telecommands
        
        Commands:
        0x01: Set transmitter power [8 bytes: double]
        0x02: Set transmitter frequency [8 bytes: double]
        0x03: Enable/disable automatic downlink [1 byte]
        0x04: Force immediate telemetry downlink [1 byte: TM type]
        """
        cmd_id = command[0]
        payload = command[1:]
        
        if cmd_id == 0x01:
            tx_power = np.frombuffer(payload, dtype=np.float64)[0]
            self.tx_power = min(tx_power, sc_cfg.COMMS_CONFIG['downlink']['tx_power'])
            logger.info(f"Comms: Transmitter power set to {self.tx_power}W")
        elif cmd_id == 0x02:
            frequency = np.frombuffer(payload, dtype=np.float64)[0]
            self.frequency = frequency
            logger.info(f"Comms: Frequency set to {self.frequency}Hz")
        elif cmd_id == 0x03:
            self.auto_downlink = bool(payload[0])
            logger.info(f"Comms: Auto downlink {'enabled' if self.auto_downlink else 'disabled'}")
        elif cmd_id == 0x04:
            self._force_telemetry_downlink(payload[0])
            logger.info(f"Comms: Forced telemetry downlink type {payload[0]}")

    def _datetime_to_cuc(self, dt: datetime) -> int:
        """Convert datetime to CUC format using mission start time as epoch"""
        delta = dt - self.epoch
        seconds = int(delta.total_seconds())
        subseconds = int((delta.total_seconds() - seconds) * 2**32)
        return (seconds << 32) | subseconds

class Payload(SubsystemBase):
    """Earth Observation Camera"""
    
    def __init__(self, spacecraft):
        super().__init__(spacecraft)
        self.imaging = False
        self.images_taken = 0
        
    def update(self) -> None:
        if self.imaging and self._check_imaging_conditions():
            self._take_image()
        
        with self.lock:
            self.telemetry.update({
                'imaging_active': self.imaging,
                'images_taken': self.images_taken
            })

    def handle_command(self, command: bytes) -> None:
        """Handle Payload telecommands
        
        Commands:
        0x01: Start/stop imaging [1 byte]
        0x02: Set image parameters [20 bytes: exposure, gain, resolution_x, resolution_y, compression]
        0x03: Take single image [0 bytes]
        0x04: Set imaging mode (SINGLE/BURST/CONTINUOUS) [1 byte]
        """
        cmd_id = command[0]
        payload = command[1:]
        
        if cmd_id == 0x01:
            self.imaging = bool(payload[0])
            logger.info(f"Payload: Imaging {'started' if self.imaging else 'stopped'}")
        elif cmd_id == 0x02:
            params = np.frombuffer(payload, dtype=np.float64)
            self.exposure = params[0]
            self.gain = params[1]
            self.resolution = (int(params[2]), int(params[3]))
            self.compression = params[4]
            logger.info(f"Payload: Image parameters updated")
        elif cmd_id == 0x03:
            if self._check_imaging_conditions():
                self._take_image()
            logger.info("Payload: Single image command received")
        elif cmd_id == 0x04:
            mode = payload[0]
            modes = {0: 'SINGLE', 1: 'BURST', 2: 'CONTINUOUS'}
            self.imaging_mode = modes.get(mode, 'SINGLE')
            logger.info(f"Payload: Imaging mode set to {self.imaging_mode}")
    
    def _check_imaging_conditions(self) -> bool:
        """Check if conditions are suitable for imaging"""
        return (not self.spacecraft.environment.is_eclipsed and
                self.spacecraft.state.power_level > 0.3)
    
    def _take_image(self) -> None:
        """Simulate taking an Earth observation image"""
        self.images_taken += 1
        self.spacecraft.send_event(
            EventSeverity.INFO,
            "PAYLOAD",
            f"Image captured successfully (#{self.images_taken})"
        )
        logger.info(f"Image captured: {self.images_taken}")

class SpacecraftSimulator:
    """Main spacecraft simulator class"""
    
    def __init__(self, universe_sim: universe.UniverseSimulator):
        self.universe = universe_sim # Universe simulator instance      
        self.dt = uni_cfg.SIM_CONFIG['time_step'] # Simulation time step (seconds)
        self.event_sequence = 0 # Event sequence number    
        self.last_status_print = datetime.now() # Time of last status print
        self.status_update_interval = 1.0 # Time between status prints (seconds)    
        self.last_status = "Initializing..." # Initialize last_status
        
        # Initialize state
        self.state = SpacecraftState()
        
        # Initialize subsystems
        self.adcs = ADCS(self)
        self.power = PowerSystem(self)
        self.comms = Communications(self)
        self.payload = Payload(self)
        
        # Start simulation thread
        self.running = True
        self.sim_thread = Thread(target=self._simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        
        logger.info("Spacecraft simulator initialized")

    def print_status(self) -> None:
        """Print status to console"""
        now = datetime.now()
        
        # Only update status string every second
        if (now - self.last_status_print).total_seconds() < self.status_update_interval:
            return
            
        self.last_status_print = now
        
        try:
            # Get state values
            pos = self.state.position / 1000.0  # Convert to km
            vel = self.state.velocity
            att = self.state.attitude
            
            # Format the mission time
            mission_time = self.state.time.strftime("%Y-%m-%d %H:%M:%S")
            
            # Format the status string
            status = (
                f"{mission_time} "  # Absolute mission time
                f"[{now.strftime('%H:%M:%S')}] "  # System time
                f"POS[{pos[0]:8.1f}, {pos[1]:8.1f}, {pos[2]:8.1f}]km "
                f"VEL[{vel[0]:6.1f}, {vel[1]:6.1f}, {vel[2]:6.1f}]km/s "
                f"ATT[{att[0]:6.3f}, {att[1]:6.3f}, {att[2]:6.3f}, {att[3]:6.3f}] "
                f"PWR:{self.state.power_level:5.1%} "
                f"TEMP:{self.state.temperature:5.1f}K "
                f"ADCS:{self.adcs.mode}({self.adcs.pointing_status}) "  # Added pointing status
                f"MODE:{self.state.mode}"
            )
            
            # Print with newline
            print(status)
                
        except Exception as e:
            error_status = f"Error generating status: {str(e)}"
            print(error_status)

    def send_event(self, severity: EventSeverity, subsystem: str, message: str) -> None:
        """Send event packet to YAMCS"""
        # Use simulation time instead of current time
        event_time = int(self.state.time.timestamp())  # Get Unix timestamp from simulation time
        
        event_data = struct.pack(
            '>HHQIB255s',
            0xE000,  # Special APID for events (0xE000)
            self.event_sequence,
            event_time,
            severity.value,
            len(message),
            message.encode('utf-8')
        )
        self.comms.send_telemetry_packet(0xE000, event_data)
        self.event_sequence = (self.event_sequence + 1) & 0x3FFF

    def handle_telecommand(self, packet: bytes) -> None:
        """Handle incoming telecommand packets"""
        # First 4 bytes are timestamp, then subsystem ID
        subsystem_id = packet[4]  # Changed from 6 to 4
        command_data = packet[5:]  # Changed from 7 to 5
        
        # Add debug output
        if len(command_data) > 0:
            logger.info(f"Received telecommand - Subsystem: 0x{subsystem_id:02x}, Command: 0x{command_data[0]:02x}, Args: {list(command_data[1:])}")
        else:
            logger.warning(f"Received telecommand with no command data - Subsystem: 0x{subsystem_id:02x}")
        
        # Route command to appropriate subsystem
        subsystem_map = {
            0x01: self.adcs,
            0x02: self.power,
            0x03: self.comms,
            0x04: self.payload
        }
        
        if subsystem_id in subsystem_map:
            try:
                subsystem_map[subsystem_id].handle_command(command_data)
            except Exception as e:
                logger.error(f"Error handling command for subsystem {subsystem_id}: {e}")
        else:
            logger.warning(f"Unknown subsystem ID: {subsystem_id}")
    
    def _simulation_loop(self) -> None:
        """Main simulation loop"""
        while self.running:
            loop_start = datetime.now()
            
            # Step the universe simulation forward
            self.universe.step()
            self.state.time = self.universe.time
            
            # Get environment state
            self.environment = self.universe.get_environment_state(
                self.state.position, self.state.velocity
            )
            
            # Update orbital dynamics
            dt = self.dt
            self.state.position += self.state.velocity * dt
            r = np.linalg.norm(self.state.position)
            if r > 0:
                g = -398600.0 / (r * r)  # Earth's gravitational parameter
                acc = g * self.state.position / r
                self.state.velocity += acc * dt
            
            # Update thermal state based on environment
            # Constants
            STEFAN_BOLTZMANN = 5.67e-8    # W/m²/K⁴
            SOLAR_FLUX = 1367.0           # W/m²
            EARTH_IR = 237.0              # W/m²
            SPACECRAFT_AREA = 0.3         # Reduced surface area (m²)
            RADIATOR_AREA = 0.1           # Reduced radiator area (m²)
            THERMAL_MASS = 100.0          # Increased thermal mass (J/K)
            EMISSIVITY = 0.8              # Radiator emissivity
            ABSORPTIVITY = 0.2            # Reduced solar absorptivity
            
            # Heat inputs
            if not self.environment.is_eclipsed:
                # Solar heating (consider spacecraft attitude)
                sun_vector = self.environment.sun_direction
                normal_vector = self.state.attitude[1:4]
                sun_angle = np.arccos(np.clip(np.dot(sun_vector, normal_vector), -1, 1))
                solar_heat = SOLAR_FLUX * ABSORPTIVITY * SPACECRAFT_AREA * abs(np.cos(sun_angle)) * 0.1  # Added scaling factor
            else:
                solar_heat = 0.0
                
            # Earth IR heating (simplified)
            earth_heat = EARTH_IR * ABSORPTIVITY * SPACECRAFT_AREA * (6371000 / r) ** 2 * 0.1  # Added scaling factor
            
            # Internal heat from power consumption (reduced)
            internal_heat = 2.0  # Reduced internal heat generation (W)
            
            # Radiative cooling (Stefan-Boltzmann law)
            temp_kelvin = self.state.temperature
            radiation_out = STEFAN_BOLTZMANN * EMISSIVITY * RADIATOR_AREA * (temp_kelvin ** 4)
            
            # Net heat and temperature change
            net_heat = solar_heat + earth_heat + internal_heat - radiation_out
            temp_change = (net_heat * dt) / THERMAL_MASS
            
            # Update temperature with bounds
            new_temp = self.state.temperature + temp_change
            self.state.temperature = np.clip(new_temp, 250.0, 350.0)  # Reasonable spacecraft temperatures
            
            # Update all subsystems
            self.adcs.update()
            self.power.update()
            self.payload.update()
            self.comms.update()
            
            # Send housekeeping telemetry
            self._send_housekeeping()
            
            # Status updates and event generation
            if self.state.temperature > 320:
                self.send_event(
                    EventSeverity.WARNING,
                    "THERMAL",
                    f"High temperature: {self.state.temperature:.1f}K"
                )
            elif self.state.temperature < 270:
                self.send_event(
                    EventSeverity.WARNING,
                    "THERMAL",
                    f"Low temperature: {self.state.temperature:.1f}K"
                )
            
            # Calculate loop timing
            processing_time = (datetime.now() - loop_start).total_seconds()
            sleep_time = max(0, self.dt - processing_time)
            sleep(sleep_time)
    
    def _send_housekeeping(self) -> None:
        """Send housekeeping telemetry packet"""
        # Calculate time delta
        time_delta = (self.state.time - self.comms.epoch).total_seconds() * 1000
        time_ms = int(time_delta)
        
        # Add timestamp to data
        time_bytes = struct.pack('>I', time_ms)
        
        # Convert ADCS mode and pointing status to integers
        adcs_mode_map = {'NADIR': 0, 'SUNPOINTING': 1, 'CUSTOM': 2}
        pointing_status_map = {'SLEWING': 0, 'POINTING': 1, 'UNKNOWN': 2}
        
        adcs_mode = adcs_mode_map.get(self.adcs.mode, 2)
        pointing_status = pointing_status_map.get(self.adcs.pointing_status, 2)
        
        # Pack the data
        state_data = struct.pack(
            '>3d3d4d3dffBB',  # Added two uint8 for mode and status
            *self.state.position,
            *self.state.velocity,
            *self.state.attitude,
            *self.state.angular_velocity,
            self.state.power_level,
            self.state.temperature,
            adcs_mode,
            pointing_status
        )
        
        # Combine time and state data
        hk_data = time_bytes + state_data
        
        self.comms.send_telemetry_packet(
            sc_cfg.CCSDS_CONFIG['apid_base'],
            hk_data
        )

    def _send_adcs_telemetry(self) -> None:
        """Send detailed ADCS telemetry packet"""
        adcs_tm = self.adcs.get_telemetry()
        adcs_data = struct.pack(
            '>4d3d3d3d3d',
            *adcs_tm['attitude'],             # q0, qx, qy, qz
            *adcs_tm['angular_velocity'],     # wx, wy, wz
            *adcs_tm['wheel_speeds'],         # wx, wy, wz
            *adcs_tm['magnetorquer_dipoles'], # mx, my, mz
            *adcs_tm['magnetic_field']        # Bx, By, Bz
        )
        self.comms.send_telemetry_packet(
            sc_cfg.CCSDS_CONFIG['apid_base'] + 1,
            adcs_data
        )

    def _send_power_telemetry(self) -> None:
        """Send detailed power telemetry packet"""
        power_tm = self.power.get_telemetry()
        power_data = struct.pack(
            '>ddd',
            power_tm['battery_charge'],       # battery charge level
            power_tm['solar_power'],          # solar panel power (W)
            power_tm['power_consumption']     # power consumption (W)
        )
        self.comms.send_telemetry_packet(
            sc_cfg.CCSDS_CONFIG['apid_base'] + 2,
            power_data
        )

    def _send_payload_telemetry(self) -> None:
        """Send detailed payload telemetry packet"""
        payload_tm = self.payload.get_telemetry()
        payload_data = struct.pack(
            '>?IQQ',
            payload_tm['imaging_active'],
            payload_tm['images_taken'],       # number of images taken
            payload_tm['storage_used'],       # storage used (bytes)
            payload_tm['storage_available']   # storage available (bytes)
        )
        self.comms.send_telemetry_packet(
            sc_cfg.CCSDS_CONFIG['apid_base'] + 3,
            payload_data
        )
    
    def stop(self) -> None:
        """Stop the simulator"""
        self.running = False
        self.sim_thread.join()
        logger.info("Spacecraft simulator stopped")
