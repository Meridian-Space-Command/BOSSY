import struct
from config import SPACECRAFT_CONFIG
import numpy as np

class PowerModule:
    def __init__(self, logger, orbit_propagator, environment, comms=None, payload=None):
        """Initialize power subsystem
        
        Args:
            logger: Logger instance
            orbit_propagator: Orbit propagator instance
            environment: Environment instance
            comms: Optional CommsModule instance
            payload: Optional PayloadModule instance
        """
        self.logger = logger
        self.orbit_propagator = orbit_propagator
        self.environment = environment
        
        # Store optional module references
        self.comms = comms
        self.payload = payload
        
        # Initialize POWER state from config
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['power']
        self.state = config['state']
        self.power_balance = config['power_balance']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.power_draw = config['power_draw']
        
        # Battery state
        self.battery_voltage = config['battery_voltage']
        self.battery_current = config['battery_current']
        
        # Power balance
        self.total_power_draw = 0.0
        self.total_power_generation = 0.0
        self.solar_panel_generation = config['solar_panel_generation'].copy()
        self.solar_panel_generation_pX = self.solar_panel_generation['pX']
        self.solar_panel_generation_nX = self.solar_panel_generation['nX']
        self.solar_panel_generation_pY = self.solar_panel_generation['pY']
        self.solar_panel_generation_nY = self.solar_panel_generation['nY']
        
    def get_telemetry(self):
        """Package current POWER state into telemetry format"""
        self.power_draw = self.power_draw + np.random.uniform(-0.05, 0.05)
        values = [
            np.uint8(self.state),                       # SubsystemState_Type (8 bits)
            np.int8(self.temperature),                  # int8_degC (8 bits)
            np.int8(self.heater_setpoint),              # int8_degC (8 bits)
            np.float32(self.power_draw),                # float_W (32 bits)
            np.float32(self.battery_voltage),           # float_V (32 bits)
            np.float32(self.battery_current),           # float_A (32 bits)
            np.uint8(self.power_balance),               # PowerBalance_Type (8 bits)
            np.float32(self.total_power_draw),          # float_W (32 bits)
            np.float32(self.total_power_generation),    # float_W (32 bits)
            np.float32(self.solar_panel_generation_pX), # float_W (32 bits)
            np.float32(self.solar_panel_generation_nX), # float_W (32 bits)
            np.float32(self.solar_panel_generation_pY), # float_W (32 bits)
            np.float32(self.solar_panel_generation_nY)  # float_W (32 bits)
        ]
        
        return struct.pack(">BbbfffBffffff", *values)
        
    def process_command(self, command_id, command_data):
        """Process POWER commands (Command_ID range 20-29)"""
        self.logger.info(f"Processing POWER command {command_id}: {command_data.hex()}")
        
        try:
            if command_id == 20:    # POWER_SET_STATE
                state = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting POWER state to: {state}")
                self.state = state
                
            elif command_id == 21:   # POWER_SET_HEATER
                heater = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting POWER heater to: {heater}")
                self.heater_state = heater
                
            elif command_id == 22:   # POWER_SET_HEATER_SETPOINT
                setpoint = struct.unpack(">f", command_data)[0]
                self.logger.info(f"Setting POWER heater setpoint to: {setpoint}°C")
                self.heater_setpoint = setpoint
                
            else:
                self.logger.warning(f"Unknown POWER command ID: {command_id}")
                
        except struct.error as e:
            self.logger.error(f"Error unpacking POWER command {command_id}: {e}")

    def process_ats_command(self, command_id, command_data):
        """Process POWER commands (Command_ID range 20-29)"""
        self.logger.info(f"Processing POWER command {command_id}: {command_data}")
        
        if command_id == 20:    # POWER_SET_STATE
            state = int(command_data)
            self.logger.info(f"Setting POWER state to: {state}")
            self.state = state
            
        elif command_id == 21:   # POWER_SET_HEATER
            heater = int(command_data)
            self.logger.info(f"Setting POWER heater to: {heater}")
            self.heater_state = heater
            
        elif command_id == 22:   # POWER_SET_HEATER_SETPOINT
            setpoint = float(command_data)
            self.logger.info(f"Setting POWER heater setpoint to: {setpoint}°C")
            self.heater_setpoint = setpoint
            
        else:
            self.logger.warning(f"Unknown POWER command ID: {command_id}")

    def update(self, current_time, adcs_module):
        """Update power state based on current conditions"""
        try:
            # Get hardware config
            power_config = SPACECRAFT_CONFIG['spacecraft']['hardware']['power']
            
            # Get orbit state
            orbit_state = self.orbit_propagator.propagate(current_time)

            # If in eclipse, no solar generation
            if orbit_state['eclipse']:
                self.solar_panel_generation = {
                    'pX': 0.0, 'nX': 0.0, 'pY': 0.0, 'nY': 0.0
                }
            else:
                # Get sun position and calculate panel angles using ADCS quaternion
                sun_pos = self.orbit_propagator.sun.get_position_at_time(current_time)
                panel_angles = self.environment.solar_illumination(
                    orbit_state['position'],
                    sun_pos,
                    adcs_module.quaternion
                )
                
                # Calculate generation for each panel
                for panel in ['pX', 'nX', 'pY', 'nY']:
                    angle = panel_angles[panel]
                    if angle < 90:  # Only generate power if angle is less than 90 degrees
                        angle_factor = np.cos(np.radians(angle))
                        power = (power_config['solar_flux'] * 
                                power_config['solar_efficiency'] * 
                                power_config['solar_panels'][panel]['area'] * 
                                angle_factor)
                    else:
                        power = 0.0
                        
                    self.solar_panel_generation[panel] = power
                    
            # Update individual panel values
            self.solar_panel_generation_pX = self.solar_panel_generation['pX']
            self.solar_panel_generation_nX = self.solar_panel_generation['nX']
            self.solar_panel_generation_pY = self.solar_panel_generation['pY']
            self.solar_panel_generation_nY = self.solar_panel_generation['nY']
            
            # Calculate total generation
            self.total_power_generation = sum(self.solar_panel_generation.values())
            
            # Simple power balance calculation
            if self.total_power_generation > self.total_power_draw:
                self.power_balance = 1  # POSITIVE
                self.battery_voltage = min(self.battery_voltage + 0.002, 8.2)
                self.battery_current = min(self.battery_current + 0.007, 0.1)
            elif self.total_power_generation < self.total_power_draw:
                self.power_balance = 2  # NEGATIVE
                self.battery_voltage = max(self.battery_voltage - 0.0005, 7.6)
                self.battery_current = max(self.battery_current - 0.007, -0.1)
            else:
                self.power_balance = 0  # BALANCED

            if hasattr(self, 'comms') and self.comms.mode == 1:
                self.battery_voltage = self.battery_voltage - 0.0002

            if hasattr(self, 'payload') and self.payload.status == 1:
                self.battery_voltage = self.battery_voltage - 0.1
            
        except Exception as e:
            self.logger.error(f"Error updating power state: {str(e)}")

    def set_total_power_draw(self, total_draw):
        """Set the total power draw from all subsystems"""
        self.total_power_draw = total_draw

    def get_power_draw(self):
        """Get the total power draw from all subsystems"""
        return self.power_draw
