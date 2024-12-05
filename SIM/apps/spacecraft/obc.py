import struct
from logger import SimLogger
from config import SPACECRAFT_CONFIG, SIM_CONFIG
import numpy as np
import time
import csv

class OBCModule:
    def __init__(self):
        self.logger = SimLogger.get_logger("OBCModule")
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['obc']
        
        # Initialize OBC state from config
        self.state = config['state']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.power_draw = config['power_draw']
        self.mode = config['mode']
        self.alive = True
        
        # Initialize time variables
        self.current_sim_time = SIM_CONFIG['mission_start_time']
        self.epoch = SIM_CONFIG['epoch']

    def update(self, current_time):
        """Update OBC state with current simulation time"""
        self.current_sim_time = current_time

    def set_subsystems(self, subsystems):
        """Set reference to subsystems dictionary"""
        self.subsystems = subsystems
        
    def get_telemetry(self):
        """Package current OBC state into telemetry format"""
        # Add small random variation to power draw
        self.power_draw = self.power_draw + np.random.uniform(-0.05, 0.05)
        
        values = [
            np.uint8(self.state),              # SubsystemState_Type (8 bits)
            np.int8(self.temperature),         # int8_degC (8 bits)
            np.int8(self.heater_setpoint),     # int8_degC (8 bits)
            np.float32(self.power_draw),       # float_W (32 bits)
            np.uint8(self.mode),                # OBCMode_Type (8 bits)
            np.uint32(self.uptime)              # uint32_s (32 bits)
        ]
        
        return struct.pack(">BbbfBI", *values)
        
    def process_command(self, command_id, command_data):
        """Process OBC commands (Command_ID range 10-19)"""
        self.logger.info(f"Processing OBC command {command_id}: {command_data.hex()}")
        
        try:
            if command_id == 10:    # OBC_SET_STATE
                state = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting OBC state to: {state}")
                self.state = state
                
            elif command_id == 11:   # OBC_SET_HEATER
                heater = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting OBC heater to: {heater}")
                self.heater_state = heater
                
            elif command_id == 12:   # OBC_SET_HEATER_SETPOINT
                setpoint = struct.unpack(">f", command_data)[0]
                self.logger.info(f"Setting OBC heater setpoint to: {setpoint}°C")
                self.heater_setpoint = setpoint
                
            elif command_id == 13:   # OBC_SET_MODE
                mode = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting OBC mode to: {mode}")
                self.mode = mode
                
            elif command_id == 14:   # OBC_RESET
                self.logger.info("Resetting OBC")
                self._reset()

            elif command_id == 15:   # OBC_START_ATS
                # Decode filename from bytes, strip null bytes
                filename = command_data.decode('utf-8').strip('\x00')
                self.logger.info(f"Starting ATS for file: {filename}")
                try:
                    # Open ats.csv file
                    with open('./apps/spacecraft/storage/ATS.csv', 'r') as file:
                        ats = csv.reader(file)
                        # make commands a list of lists
                        commands = [list(row) for row in ats]
                    self.process_ats(commands)
                except Exception as e:
                    self.logger.error(f"Error importing ATS: {e}")
                    
                else:
                    self.logger.warning(f"Unknown OBC command ID: {command_id}")
                
        except struct.error as e:
            self.logger.error(f"Error unpacking OBC command {command_id}: {e}")

    def process_ats_command(self, command_id, command_data):
        """Process OBC commands (Command_ID range 10-19)"""
        self.logger.info(f"Processing OBC command {command_id}: {command_data}")
        
        if command_id == 10:    # OBC_SET_STATE
            state = int(command_data)
            self.logger.info(f"Setting OBC state to: {state}")
            self.state = state
            
        elif command_id == 11:   # OBC_SET_HEATER
            heater = int(command_data)
            self.logger.info(f"Setting OBC heater to: {heater}")
            self.heater_state = heater
            
        elif command_id == 12:   # OBC_SET_HEATER_SETPOINT
            setpoint = float(command_data)
            self.logger.info(f"Setting OBC heater setpoint to: {setpoint}°C")
            self.heater_setpoint = setpoint
            
        elif command_id == 13:   # OBC_SET_MODE
            mode = int(command_data)
            self.logger.info(f"Setting OBC mode to: {mode}")
            self.mode = mode
            
        elif command_id == 14:   # OBC_RESET
            self.logger.info("Resetting OBC")
            self._reset()

        elif command_id == 15:   # OBC_START_ATS
            filename = command_data
            self.logger.info(f"Starting ATS for file: {filename}")
            try:
                # Open ats.csv file
                with open(f'./apps/spacecraft/storage/{filename}', 'r') as file:
                    ats = csv.reader(file)
                    # make commands a list of lists
                    commands = [list(row) for row in ats]
                self.process_ats(commands)
            except Exception as e:
                self.logger.error(f"Error importing ATS: {e}")

        else:
            self.logger.warning(f"Unknown OBC command ID: {command_id}")
                
    def process_ats(self, commands):
        """Process ATS commands"""
        while self.alive:
            for command in commands:
                command[0] = int(command[0])
                command[1] = int(command[1])
                command_time_delta = command[0] - int((self.current_sim_time - self.epoch).total_seconds())
                self.logger.debug(f"Current time: {self.current_sim_time}")
                self.logger.debug(f"Epoch: {self.epoch}")
                self.logger.debug(f"Current time delta: {int((self.current_sim_time - self.epoch).total_seconds())}")
                self.logger.debug(f"Command time: {command[0]}")
                self.logger.debug(f"Command time delta: {command_time_delta}")
                if command_time_delta < 0:
                    self.logger.warning(f"ATS command {command[1]} at {command[0]} is in the past")
                else:
                    while command_time_delta > 0:
                        time.sleep(SIM_CONFIG['time_step'])
                        self.logger.debug(f"Time to command_id {command[1]}: {command_time_delta} seconds")
                        command_time_delta = command[0] - int((self.current_sim_time - self.epoch).total_seconds())
                        
                    if 10 <= command[1] <= 19:
                        self.process_ats_command(command[1], command[2])
                    else:
                        # Route to appropriate subsystem's ats_command processor
                        if 20 <= command[1] <= 29:
                            self.subsystems['power'].process_ats_command(command[1], command[2])
                        elif 30 <= command[1] <= 39:
                            self.subsystems['adcs'].process_ats_command(command[1], command[2])
                        elif 40 <= command[1] <= 49:
                            self.subsystems['comms'].process_ats_command(command[1], command[2])
                        elif 50 <= command[1] <= 59:
                            self.subsystems['payload'].process_ats_command(command[1], command[2])
                        elif 60 <= command[1] <= 69:
                            self.subsystems['datastore'].process_ats_command(command[1], command[2])
                        else:
                            self.logger.warning(f"Unhandled command ID: {command[1]}")

    def _reset(self):
        """Reset OBC to initial state"""
        self.alive = False
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['obc']
        self.state = config['state']
        self.mode = config['mode']
        self.heater_setpoint = config['heater_setpoint']
        self.power_draw = config['power_draw']
        self.uptime = config['uptime']
        self.alive = True
        self.logger.info("OBC reset complete")

    def get_mode(self):
        """Get the current OBC mode"""
        return self.mode

    def get_power_draw(self):
        """Get current power draw in Watts"""
        return self.power_draw
    
    def set_uptime(self, uptime):
        """Set the uptime in seconds"""
        self.uptime = uptime
        
    def get_uptime(self):
        """Get the uptime in seconds"""
        return self.uptime

    def update(self, current_time):
        """Update OBC state with current simulation time"""
        self.current_sim_time = current_time