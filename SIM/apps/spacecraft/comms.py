import struct
import socket
import threading
from logger import SimLogger
from config import SPACECRAFT_CONFIG
import numpy as np

class CommsModule:
    def __init__(self):
        self.logger = SimLogger.get_logger("CommsModule")
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['comms']
        comms_config = SPACECRAFT_CONFIG['spacecraft']['comms']
        
        # Initialize COMMS state from config
        self.state = config['state']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.rx_power_draw = config['rx_power_draw']
        self.txrx_power_draw = config['txrx_power_draw']
        self.power_draw = self.txrx_power_draw
        self.mode = config['mode']
        
        # Communication parameters
        self.packets_sent = 0
        self.packets_received = 0
        self.uplink_bitrate = config['uplink_bitrate']
        self.downlink_bitrate = config['downlink_bitrate']
        
        # Socket configuration
        self.tc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.TC_PORT = comms_config['tc_port']
        self.TM_PORT = comms_config['tm_port']
        self.HOST = comms_config['host']
        
        self.running = False
        self.cdh = None  # Will be set later via set_cdh
        
    def set_cdh(self, cdh):
        """Set reference to CDH module"""
        self.cdh = cdh
        
    def set_subsystems(self, subsystems):
        """Set reference to subsystems dictionary"""
        self.subsystems = subsystems
        
    def get_telemetry(self):
        """Package current COMMS state into telemetry format"""
        if self.mode == 0:
            self.power_draw = self.rx_power_draw + np.random.uniform(-0.05, 0.05)
        else:
            self.power_draw = self.txrx_power_draw + np.random.uniform(-0.05, 0.05)

        values = [
            np.uint8(self.state),              # SubsystemState_Type (8 bits)
            np.int8(self.temperature),         # int8_degC (8 bits)
            np.int8(self.heater_setpoint),     # int8_degC (8 bits)
            np.float32(self.power_draw),       # float_W (32 bits)
            np.uint8(self.mode),               # CommsMode_Type (8 bits)
            np.uint32(self.downlink_bitrate),        # uint32_bps (32 bits)
            np.uint32(self.uplink_bitrate)         # uint32_bps (32 bits)
        ]
        
        return struct.pack(">BbbfBII", *values)
        
    def process_command(self, command_id, command_data):
        """Process COMMS commands (Command_ID range 40-49)"""
        self.logger.info(f"Processing COMMS command {command_id}: {command_data.hex()}")
        
        try:
            if command_id == 40:    # COMMS_SET_STATE
                state = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting COMMS state to: {state}")
                self.state = state
                
            elif command_id == 41:   # COMMS_SET_HEATER
                heater = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting COMMS heater to: {heater}")
                self.heater_state = heater
                
            elif command_id == 42:   # COMMS_SET_HEATER_SETPOINT
                setpoint = struct.unpack(">f", command_data)[0]
                self.logger.info(f"Setting COMMS heater setpoint to: {setpoint}°C")
                self.heater_setpoint = setpoint
                
            elif command_id == 43:   # COMMS_SET_MODE
                mode = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting COMMS mode to: {mode}")
                self.mode = mode
                
            else:
                self.logger.warning(f"Unknown COMMS command ID: {command_id}")
                
        except struct.error as e:
            self.logger.error(f"Error unpacking COMMS command {command_id}: {e}")

    def process_ats_command(self, command_id, command_data):
        """Process COMMS commands (Command_ID range 40-49)"""
        self.logger.info(f"Processing COMMS command {command_id}: {command_data}")    
        if command_id == 40:    # COMMS_SET_STATE
            state = int(command_data)
            self.logger.info(f"Setting COMMS state to: {state}")
            self.state = state
            
        elif command_id == 41:   # COMMS_SET_HEATER
            heater = int(command_data)
            self.logger.info(f"Setting COMMS heater to: {heater}")
            self.heater_state = heater
            
        elif command_id == 42:   # COMMS_SET_HEATER_SETPOINT
            setpoint = float(command_data)  
            self.logger.info(f"Setting COMMS heater setpoint to: {setpoint}°C")
            self.heater_setpoint = setpoint
            
        elif command_id == 43:   # COMMS_SET_MODE
            mode = int(command_data)
            self.logger.info(f"Setting COMMS mode to: {mode}")
            self.mode = mode
            
        else:
            self.logger.warning(f"Unknown COMMS command ID: {command_id}")
            
    def start(self):
        """Start the communications module"""
        self.running = True
        self.tc_socket.bind((self.HOST, self.TC_PORT))
        
        # Start TC listener thread
        self.tc_thread = threading.Thread(target=self._tc_listener)
        self.tc_thread.daemon = True
        self.tc_thread.start()
        
        self.logger.info(f"CommsModule started - Listening for telecommands on port {self.TC_PORT}")
        
    def stop(self):
        """Stop the communications module"""
        self.running = False
        try:
            self.tc_socket.shutdown(socket.SHUT_RDWR)
            self.tc_socket.close()
            self.tm_socket.close()
        except Exception as e:
            self.logger.debug(f"Socket cleanup error (expected during shutdown): {e}")
        self.logger.info("CommsModule stopped")
        
    def send_tm_packet(self, packet):
        """Send telemetry packet"""
        if self.mode == 1:  # if TXRX mode, send to TM socket
            try:
                self.tm_socket.sendto(packet, (self.HOST, self.TM_PORT))
                self.packets_sent += 1
                self.logger.debug(f"Sent TM packet: {packet.hex()}")
            except socket.error as e:
                self.logger.error(f"Socket error while sending TM: {e}")
            
    def _tc_listener(self):
        """Listen for telecommands on UDP socket"""
        while True:
            try:
                data, addr = self.tc_socket.recvfrom(1024)
                self.logger.info(f"Received TC from {addr}")
                self.logger.debug(f"Raw data: {data.hex()}")
                
                # Skip CCSDS header (6 bytes) and time (4 bytes)
                header_size = 10
                
                # Parse command ID from next 2 bytes
                command_id = struct.unpack(">H", data[header_size:header_size+2])[0]
                command_data = data[header_size+2:]
                
                self.logger.debug(f"Parsed command ID: {command_id}, data: {command_data.hex()}")
                
                # Route command through CDH
                self.cdh.route_command(command_id, command_data, self.subsystems)
                
            except Exception as e:
                self.logger.error(f"Error in TC listener: {e}")

    def set_mode(self, mode_requested):
        """Set the COMMS mode"""
        self.mode = mode_requested

    def get_power_draw(self):
        """Get current power draw in Watts"""
        return self.power_draw
