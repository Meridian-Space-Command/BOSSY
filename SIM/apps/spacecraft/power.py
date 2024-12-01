import struct
from config import SPACECRAFT_CONFIG
import numpy as np
from .adcs import ADCSModule
from astropy import units as u

class PowerModule:
    def __init__(self, logger, orbit_propagator, environment, comms=None, payload=None, datastore=None):
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
        self.datastore = datastore
        
        # Initialize POWER state from config
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['power']
        comms_config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['comms']
        
        # State values
        self.state = config['state']
        self.power_balance = config['power_balance']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.power_draw = config['power_draw']
        self.mode = 0  # Default mode
        
        # Power draw values from comms config
        self.rx_power_draw = comms_config['rx_power_draw']
        self.txrx_power_draw = comms_config['txrx_power_draw']
        
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
        if self.mode == 0:
            self.power_draw = (self.rx_power_draw * u.W + np.random.uniform(-0.05, 0.05) * u.W).value
        else:
            self.power_draw = (self.txrx_power_draw * u.W + np.random.uniform(-0.05, 0.05) * u.W).value

        values = [
            np.uint8(self.state),
            np.int8(self.temperature),
            np.int8(self.heater_setpoint),
            np.float32(self.power_draw),
            np.float32(self.battery_voltage),
            np.float32(self.battery_current),
            np.uint8(self.power_balance),
            np.float32(self.total_power_draw),
            np.float32(self.total_power_generation),
            np.float32(self.solar_panel_generation_pX),
            np.float32(self.solar_panel_generation_nX),
            np.float32(self.solar_panel_generation_pY),
            np.float32(self.solar_panel_generation_nY)
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
            # Get hardware config with units
            power_config = SPACECRAFT_CONFIG['spacecraft']['hardware']['power']
            solar_flux = power_config['solar_flux'] * u.W / u.m**2
            solar_efficiency = power_config['solar_efficiency'] * u.dimensionless_unscaled
            
            # Get orbit state
            orbit_state = self.orbit_propagator.propagate(current_time)
            self.logger.debug(f"Orbit state: {orbit_state}")
            
            # Calculate power generation
            total_generation = 0.0 * u.W  # Initialize with unit
            
            # Check eclipse status
            eclipse_status = orbit_state['eclipse']
            self.logger.debug(f"Eclipse status: {eclipse_status}")
            
            if not eclipse_status['in_eclipse']:
                # Get panel angles from environment
                panel_angles = self.environment.solar_illumination(
                    orbit_state,
                    None,
                    adcs_module.quaternion
                )
                
                self.logger.debug(f"Panel angles: {panel_angles}")
                
                for panel in ['pX', 'nX', 'pY', 'nY']:
                    area = power_config['solar_panels'][panel]['area'] * u.m**2
                    angle = float(panel_angles[panel])  # Ensure it's a float
                    
                    self.logger.debug(f"Panel {panel}: angle={angle} degrees, area={area}")
                    
                    if angle < 90.0:  # Compare floats
                        angle_rad = np.radians(angle)
                        angle_factor = np.cos(angle_rad)
                        power = (solar_flux * solar_efficiency * area * angle_factor)
                        total_generation += power
                        self.solar_panel_generation[panel] = power.to(u.W).value
                        self.logger.debug(f"Panel {panel} generating {power.to(u.W):.2f}")
                    else:
                        self.solar_panel_generation[panel] = 0.0
                        self.logger.debug(f"Panel {panel} not generating power (angle={angle})")
            else:
                self.logger.debug("In eclipse - no power generation")
                for panel in ['pX', 'nX', 'pY', 'nY']:
                    self.solar_panel_generation[panel] = 0.0
            
            self.total_power_generation = total_generation.to(u.W).value
            self.logger.debug(f"Total power generation: {self.total_power_generation:.2f} W")
            
            # Update individual panel values
            self.solar_panel_generation_pX = self.solar_panel_generation['pX']
            self.solar_panel_generation_nX = self.solar_panel_generation['nX']
            self.solar_panel_generation_pY = self.solar_panel_generation['pY']
            self.solar_panel_generation_nY = self.solar_panel_generation['nY']
            
            # Calculate total generation
            self.total_power_generation = sum(self.solar_panel_generation.values())
            
            # Simple power balance calculation
            total_generation = self.total_power_generation * u.W  # Add units
            total_draw = self.total_power_draw * u.W  # Add units
            
            if total_generation.value > total_draw.value:  # Compare values without units
                self.power_balance = 1  # POSITIVE
                if adcs_module.is_sunpointing:
                    self.battery_voltage = min(self.battery_voltage + 0.0005, 8.2)
                    self.battery_current = min(self.battery_current + 0.005, 0.1)
                else:
                    self.battery_voltage = min(self.battery_voltage + 0.0001, 8.2)
                    self.battery_current = min(self.battery_current + 0.005, 0.1)
            elif total_generation.value < total_draw.value:  # Compare values without units
                self.power_balance = 2  # NEGATIVE
                self.battery_voltage = max(self.battery_voltage - 0.0003, 6.5)
                self.battery_current = max(self.battery_current - 0.004, -0.1)
            else:
                self.power_balance = 0  # BALANCED

            # Subsystem power effects
            if hasattr(self, 'comms') and self.comms.mode == 1:
                self.battery_voltage = self.battery_voltage - 0.0002

            if hasattr(self, 'payload') and self.payload.status == 1:
                self.battery_voltage = self.battery_voltage - 0.1

            if hasattr(self, 'datastore') and self.datastore.mode == 1:
                self.battery_voltage = self.battery_voltage - 0.2
            
            self.logger.debug(f"Power balance: {self.power_balance}, Battery: {self.battery_voltage:.2f}V @ {self.battery_current:.3f}A")
            
        except Exception as e:
            self.logger.error(f"Error updating power state: {str(e)}")

    def set_total_power_draw(self, total_draw):
        """Set the total power draw from all subsystems"""
        self.total_power_draw = total_draw

    def get_power_draw(self):
        """Get the total power draw from all subsystems"""
        return self.power_draw
