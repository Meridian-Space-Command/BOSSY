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
from datetime import datetime, timedelta
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
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class SpacecraftState:
    """Complete spacecraft state representation"""
    time: datetime
    position: np.ndarray          # [x, y, z] in ECI (m)
    velocity: np.ndarray          # [vx, vy, vz] in ECI (m/s)
    attitude: np.ndarray          # Quaternion [w, x, y, z]
    angular_velocity: np.ndarray  # [wx, wy, wz] (rad/s)
    power_level: float           # Battery charge level (0-1)
    temperature: float           # Average temperature (K)
    mode: str                    # Spacecraft operation mode

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
        """Handle telecommands for this subsystem"""
        raise NotImplementedError

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
        self.wheel_speeds = np.zeros(3)
        self.magnetorquer_dipoles = np.zeros(3)
        self.mode = 'NOMINAL'
        
    def update(self) -> None:
        env = self.spacecraft.environment
        
        # Attitude determination (simplified)
        # In practice, use proper sensor fusion algorithms
        measured_mag = env.magnetic_field + np.random.normal(0, 1e-7, 3)
        
        # Control law (simple PD controller)
        q_error = self._quaternion_error(self.spacecraft.state.attitude, self.target_attitude)
        torque = -0.001 * q_error[1:] - 0.0001 * self.spacecraft.state.angular_velocity
        
        # Actuator commands
        self.wheel_speeds += torque * self.spacecraft.dt
        self.wheel_speeds = np.clip(self.wheel_speeds, 
                                  -sc_cfg.ADCS_CONFIG['reaction_wheels']['max_momentum'],
                                  sc_cfg.ADCS_CONFIG['reaction_wheels']['max_momentum'])
        
        # Update telemetry
        with self.lock:
            self.telemetry.update({
                'attitude': self.spacecraft.state.attitude.tolist(),
                'angular_velocity': self.spacecraft.state.angular_velocity.tolist(),
                'wheel_speeds': self.wheel_speeds.tolist(),
                'magnetorquer_dipoles': self.magnetorquer_dipoles.tolist(),
                'magnetic_field': measured_mag.tolist()
            })
    
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
            modes = {0: 'NADIR', 1: 'SUNPOINTING', 2: 'CUSTOM'}
            self.control_mode = modes.get(mode, 'CUSTOM')
            logger.info(f"ADCS: Control mode set to {self.control_mode}")
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

class PowerSystem(SubsystemBase):
    """Power System"""
    
    def __init__(self, spacecraft):
        super().__init__(spacecraft)
        self.battery_charge = sc_cfg.POWER_CONFIG['battery']['initial_charge']
        self.solar_panel_power = 0.0
        
    def update(self) -> None:
        env = self.spacecraft.environment
        
        # Solar panel power generation
        panel_area = sc_cfg.POWER_CONFIG['solar_panels']['area_per_panel']
        efficiency = sc_cfg.POWER_CONFIG['solar_panels']['efficiency']
        self.solar_panel_power = panel_area * efficiency * env.solar_flux
        
        # Power consumption
        consumption = sum(sc_cfg.POWER_CONFIG['power_consumption'].values())
        
        # Battery charge update
        battery_capacity = sc_cfg.POWER_CONFIG['battery']['capacity']
        charge_delta = (self.solar_panel_power - consumption) * self.spacecraft.dt / (battery_capacity * 3600)
        self.battery_charge = np.clip(self.battery_charge + charge_delta, 0, 1)
        
        # Update telemetry
        with self.lock:
            self.telemetry.update({
                'battery_charge': self.battery_charge,
                'solar_power': self.solar_panel_power,
                'power_consumption': consumption
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
        
    def send_telemetry_packet(self, apid: int, data: bytes) -> None:
        """Send CCSDS Space Packet"""
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
        self.state = SpacecraftState(
            time=self.universe.time,
            position=np.array([7000.0, 0.0, 0.0]),  # Start at 7000km altitude
            velocity=np.array([0.0, 7.5, 0.0]),     # Initial orbital velocity
            attitude=np.array([1., 0., 0., 0.]),    # Initial attitude (nominal)    
            angular_velocity=np.zeros(3),           # Zero angular velocity  
            power_level=1.0,                        # 100% power level
            temperature=293.0,                      # 20°C
            mode='NOMINAL'                          # Nominal mode
        )
        
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

    def print_status(self) -> str:
        """Returns a status string for console output"""
        now = datetime.now()
        
        # Only update status string every second
        if (now - self.last_status_print).total_seconds() < self.status_update_interval:
            return self.last_status
            
        self.last_status_print = now
        
        try:
            # Get state values
            pos = self.state.position / 1000.0  # Convert to km
            vel = self.state.velocity
            att = self.state.attitude
            
            # Format the status string
            status = (
                f"POS[{pos[0]:8.1f}, {pos[1]:8.1f}, {pos[2]:8.1f}]km "
                f"VEL[{vel[0]:6.1f}, {vel[1]:6.1f}, {vel[2]:6.1f}]km/s "
                f"ATT[{att[0]:6.3f}, {att[1]:6.3f}, {att[2]:6.3f}, {att[3]:6.3f}] "
                f"PWR:{self.state.power_level:5.1%} "
                f"TEMP:{self.state.temperature:5.1f}K "
                f"ADCS:{getattr(self.adcs, 'mode', 'UNKNOWN')} "  # Safe access to mode
                f"MODE:{self.state.mode}"
            )
            
            self.last_status = status
            return status
            
        except Exception as e:
            error_status = f"Error generating status: {str(e)}"
            self.last_status = error_status
            return error_status

    def send_event(self, severity: EventSeverity, subsystem: str, message: str) -> None:
        """Send event packet to YAMCS"""
        event_time = self.state.time.timestamp()
        event_data = struct.pack(
            '>HHQIB255s',
            0xE000,  # Special APID for events (0xE000)
            self.event_sequence,
            int(event_time),
            severity.value,
            len(message),
            message.encode('utf-8')
        )
        self.comms.send_telemetry_packet(0xE000, event_data)
        self.event_sequence = (self.event_sequence + 1) & 0x3FFF

    def handle_telecommand(self, packet: bytes) -> None:
        """Handle incoming telecommand packets
        
        CCSDS Packet Structure:
        - Primary Header (6 bytes)
        - Secondary Header (1 byte): Subsystem ID
        - Command Data
        """
        # Extract subsystem ID from secondary header
        subsystem_id = packet[6]
        command_data = packet[7:]
        
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
            
            # Get environment state
            self.environment = self.universe.get_environment_state(
                self.state.position,     # x, y, z
                self.state.velocity      # vx, vy, vz 
            )
            
            # Update all subsystems
            self.adcs.update()
            self.power.update()
            self.payload.update()
            
            # Send housekeeping telemetry
            self._send_housekeeping()

            if self.state.power_level < 0.2:
                self.send_event(
                    EventSeverity.WARNING,
                    "POWER",
                    f"Low battery level: {self.state.power_level:.2%}"
                )
            
            if self.state.temperature > 320:  # 320K = ~47°C
                self.send_event(
                    EventSeverity.ERROR,
                    "THERMAL",
                    f"High temperature: {self.state.temperature:.1f}K"
                )
            
            # Calculate loop timing
            processing_time = (datetime.now() - loop_start).total_seconds()
            sleep_time = max(0, self.dt - processing_time)
            sleep(sleep_time)
    
    def _send_housekeeping(self) -> None:
        """Send housekeeping telemetry packet"""
        # Main housekeeping packet (APID = base)
        hk_data = struct.pack(
            '>3d3d4d3dff',
            *self.state.position,         # x, y, z       
            *self.state.velocity,         # vx, vy, vz
            *self.state.attitude,         # q0, qx, qy, qz
            *self.state.angular_velocity, # wx, wy, wz
            self.state.power_level,       # battery charge level
            self.state.temperature        # average temperature (K) 
        )
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