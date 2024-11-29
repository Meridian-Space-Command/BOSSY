import struct
from logger import SimLogger
from config import SPACECRAFT_CONFIG, SIM_CONFIG
import numpy as np

class CDHModule:
    def __init__(self):
        self.logger = SimLogger.get_logger("CDHModule")
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['cdh']
        self.epoch = SIM_CONFIG['epoch']
        
        # Initialize CDH state from config
        self.state = config['state']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.power_draw = config['power_draw']
        
        # Initialize sequence counter for CCSDS packets
        self.sequence_count = 0
        
    def get_telemetry(self):
        """Package CDH state into telemetry format"""
        self.power_draw = self.power_draw + np.random.uniform(-0.05, 0.05)
        values = [
            np.uint8(self.state),              # SubsystemState_Type (8 bits)
            np.int8(self.temperature),         # int8_degC (8 bits)
            np.int8(self.heater_setpoint),     # int8_degC (8 bits)
            np.float32(self.power_draw)        # float_W (32 bits)
        ]
        
        return struct.pack(">Bbbf", *values)

    def create_tm_packet(self, current_sim_time, subsystems):
        """Create a CCSDS telemetry packet"""
        # CCSDS Primary Header
        version = 0
        packet_type = 0  # TM
        sec_hdr_flag = 1  # Enable secondary header
        apid = 100  # Housekeeping
        sequence_flags = 3  # Standalone packet
        packet_sequence_count = self.sequence_count & 0x3FFF
    
        first_word = (version << 13) | (packet_type << 12) | (sec_hdr_flag << 11) | apid
        second_word = (sequence_flags << 14) | packet_sequence_count
        
        # Calculate elapsed time in seconds since mission start
        elapsed_seconds = int((current_sim_time - self.epoch).total_seconds())
        
        # Pack as 4-byte unsigned int
        secondary_header = struct.pack(">I", elapsed_seconds & 0xFFFFFFFF)
        
        # Get telemetry from all subsystems in correct order
        data = (
            subsystems['obc'].get_telemetry() +
            self.get_telemetry() +
            subsystems['power'].get_telemetry() +
            subsystems['adcs'].get_telemetry() +
            subsystems['comms'].get_telemetry() +
            subsystems['payload'].get_telemetry() +
            subsystems['datastore'].get_telemetry()
        )
        
        # Calculate packet length (minus 1 per CCSDS standard)
        packet_length = len(secondary_header) + len(data) - 1
        
        # Create the complete packet
        packet = struct.pack(">HHH", first_word, second_word, packet_length) + secondary_header + data
        
        self.sequence_count += 1
        return packet

    def route_command(self, command_id, command_data, subsystems):
        """Route commands to appropriate subsystem"""
        try:            
            self.logger.info(f"Routing command ID: {command_id}")
            
            # Route commands to appropriate subsystem
            if 10 <= command_id <= 19:
                subsystems['obc'].process_command(command_id, command_data)
            elif 20 <= command_id <= 29:
                subsystems['power'].process_command(command_id, command_data)
            elif 30 <= command_id <= 39:
                subsystems['adcs'].process_command(command_id, command_data)
            elif 40 <= command_id <= 49:
                subsystems['comms'].process_command(command_id, command_data)
            elif 50 <= command_id <= 59:
                subsystems['payload'].process_command(command_id, command_data)
            elif 60 <= command_id <= 69:
                subsystems['datastore'].process_command(command_id, command_data)
            else:
                self.logger.warning(f"Unhandled command ID: {command_id}")
                
        except struct.error as e:
            self.logger.error(f"Error processing command: {e}")

    def get_power_draw(self):
        """Get current power draw in Watts"""
        return self.power_draw
