import numpy as np
from scipy.spatial.transform import Rotation
from astropy import units as u
from ..universe.environment import Environment
from ..universe.orbit import OrbitPropagator
from config import SPACECRAFT_CONFIG, SIM_CONFIG
from logger import SimLogger
import struct

class ADCSModule:
    # ADCS modes
    MODES = ['OFF', 'DETUMBLE', 'SUNPOINTING', 'NADIR', 'DOWNLOAD', 'EOL']
    
    def __init__(self, initial_state=None, orbit_propagator=None):
        self.logger = SimLogger.get_logger("ADCSModule")
        
        # Use provided orbit propagator or create new one
        self.orbit_propagator = orbit_propagator or OrbitPropagator()
        
        # Initialize environment
        self.environment = Environment()
        
        # Initialize from config
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['adcs']
        self.state = config['state']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.power_draw = config['power_draw']
        self.mode = config['mode']
        self.status = config['status']

        # Get configuration values
        self.dt = SIM_CONFIG['time_step']
        adcs_config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['adcs']
        adcs_hw_config = SPACECRAFT_CONFIG['spacecraft']['hardware']['adcs']
        
        # Initialize attitude state with proper units and reasonable initial rates
        self.quaternion = np.array(adcs_config['quaternion'])
        self.angular_rate = np.array(adcs_config['angular_rate'])
        self.angular_rate_total = 0.0
        
        # Initialize mode and status from config
        self.mode = adcs_config['mode']
        self.status = adcs_config['status']
        self._burn_initiated = False
        
        # Get pointing requirements from config
        self.accuracy_threshold = adcs_hw_config['pointing_requirements']['accuracy_threshold']
        self.stability_duration = adcs_hw_config['pointing_requirements']['stability_duration']
        self.max_slew_rate = adcs_hw_config['pointing_requirements']['max_slew_rate']
        self.nominal_slew_rate = adcs_hw_config['pointing_requirements']['nominal_slew_rate']
        
        if initial_state:
            # Initialize position from orbit state
            self.position = [
                initial_state['lat'],   # degrees
                initial_state['lon'],   # degrees
                initial_state['alt']    # km
            ]
            self.eclipse = 1 if initial_state['eclipse']['in_eclipse'] else 0
        else:
            # Use config defaults
            self.position = np.array(config['position'])
            self.eclipse = config['eclipse']
        
        # Track last update time
        self.last_update_time = SIM_CONFIG['mission_start_time']
        
        # Get hardware config
        hw_config = SPACECRAFT_CONFIG['spacecraft']['hardware']['adcs']
        self.pointing_accuracy = hw_config['pointing_requirements']['accuracy_threshold']
        self.pointing_duration = hw_config['pointing_requirements']['stability_duration']
        self.pointing_start_time = None
        
        self.time_step = SIM_CONFIG['time_step']
        
        self.logger.info("ADCS Module initialized")
        
    def update(self, current_time, orbit_state):
        """Update ADCS state"""
        try:
            # Update position and eclipse state directly from orbit state
            self.position = [
                orbit_state['lat'],
                orbit_state['lon'],
                orbit_state['alt']
            ]
            self.eclipse = 1 if orbit_state['eclipse']['in_eclipse'] else 0
            
            # Calculate environmental effects
            self.density = self.environment.atmospheric_density(orbit_state['alt'])
            
            # Update attitude based on mode
            self._update_attitude(orbit_state)
            
            self.last_update_time = current_time
            
            self.logger.debug(f"ADCS state updated - q: {self.quaternion}")
            
        except Exception as e:
            self.logger.error(f"Error updating ADCS state: {str(e)}")
            self.status = 0  # Set to UNCONTROLLED
        
    def get_telemetry(self):
        """Package current ADCS state into telemetry format"""
        # Convert angular rates to deg/s for telemetry
        rates_deg_s = self.angular_rate  # Already in deg/s
        rate_total = np.linalg.norm(rates_deg_s)
        
        values = [
            np.uint8(self.state),
            np.int8(self.temperature),
            np.int8(self.heater_setpoint),
            np.float32(self.power_draw),
            np.uint8(self.mode),
            np.uint8(self.status),
            np.float32(self.quaternion[0]),
            np.float32(self.quaternion[1]),
            np.float32(self.quaternion[2]),
            np.float32(self.quaternion[3]),
            np.float32(rates_deg_s[0]),
            np.float32(rates_deg_s[1]),
            np.float32(rates_deg_s[2]),
            np.float32(rate_total),
            np.float32(self.position[0]),
            np.float32(self.position[1]),
            np.float32(self.position[2]),
            np.uint8(self.eclipse)
        ]
        
        return struct.pack(">BbbfBBfffffffffffB", *values)
        
    def process_command(self, command_id, command_data):
        """Process ADCS commands (Command_ID range 30-39)"""
        self.logger.info(f"Processing ADCS command {command_id}: {command_data.hex()}")
        
        try:
            if command_id == 30:    # ADCS_SET_STATE
                state = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting ADCS state to: {state}")
                self.state = state
                
            elif command_id == 31:   # ADCS_SET_HEATER
                heater = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting ADCS heater to: {heater}")
                self.heater_state = heater
                
            elif command_id == 32:   # ADCS_SET_HEATER_SETPOINT
                setpoint = struct.unpack(">f", command_data)[0]
                self.logger.info(f"Setting ADCS heater setpoint to: {setpoint}°C")
                self.heater_setpoint = setpoint
                
            elif command_id == 33:  # Set ADCS mode
                new_mode = struct.unpack('B', command_data)[0]
                self.logger.info(f"Received request to change to mode: {new_mode}")
                
                # Validate mode
                if new_mode > 5:
                    self.logger.error(f"Invalid ADCS mode: {new_mode}")
                    return
                
                # Update the mode directly instead of using requested_mode
                self.mode = new_mode  # Add this line
                self.status = 1  # SLEWING
                self.pointing_start_time = None
                self.logger.info(f"Beginning slew to requested mode {new_mode}")

            elif command_id == 34:  # ADCS_DEORBIT_BURN
                self.logger.info("Initiating deorbit burn")
                if self.mode == 5 and self.status == 2:
                    self._burn_initiated = True
                else:
                    self.logger.error("Deorbit burn can only be initiated in ADCS mode = EOL and status = POINTING.")
                
            else:
                self.logger.warning(f"Unknown ADCS command ID: {command_id}")
                
        except struct.error as e:
            self.logger.error(f"Error unpacking ADCS command {command_id}: {e}")

    def process_ats_command(self, command_id, command_data):
        """Process ADCS commands (Command_ID range 30-39)"""
        self.logger.info(f"Processing ADCS command {command_id}: {command_data}")
        
        if command_id == 30:    # ADCS_SET_STATE
            state = int(command_data)
            self.logger.info(f"Setting ADCS state to: {state}")
            self.state = state
            
        elif command_id == 31:   # ADCS_SET_HEATER
            heater = int(command_data)
            self.logger.info(f"Setting ADCS heater to: {heater}")
            self.heater_state = heater
            
        elif command_id == 32:   # ADCS_SET_HEATER_SETPOINT
            setpoint = float(command_data)
            self.logger.info(f"Setting ADCS heater setpoint to: {setpoint}°C")
            self.heater_setpoint = setpoint
            
        elif command_id == 33:  # Set ADCS mode
            new_mode = int(command_data)
            self.logger.info(f"Received request to change to mode: {new_mode}")
            
            # Validate mode
            if new_mode > 5:
                self.logger.error(f"Invalid ADCS mode: {new_mode}")
                return
            
            # Update the mode directly instead of using requested_mode
            self.mode = new_mode  # Add this line
            self.status = 1  # SLEWING
            self.pointing_start_time = None
            self.logger.info(f"Beginning slew to requested mode {new_mode}")

        elif command_id == 34:  # ADCS_DEORBIT_BURN
            self.logger.info("Initiating deorbit burn")
            if self.mode == 5 and self.status == 2:
                self._burn_initiated = True
            else:
                self.logger.error("Deorbit burn can only be initiated in ADCS mode = EOL and status = POINTING.")
            
        else:
            self.logger.warning(f"Unknown ADCS command ID: {command_id}")
                

    def _update_attitude(self, orbit_state):
        """Update spacecraft attitude based on current mode"""
        try:
            control_mode = self.mode
            q_desired = None

            # Calculate desired quaternion based on mode
            if control_mode == 0:  # OFF/UNCONTROLLED
                self._uncontrolled_motion()
                return
            
            elif control_mode == 1:  # DETUMBLE
                self._detumble()
                return
            
            elif control_mode == 2:  # SUNPOINTING
                # Calculate sun pointing quaternion
                q_desired = self._calculate_sun_pointing(orbit_state, self.last_update_time)
            
            elif control_mode == 3:  # NADIR
                # Calculate nadir pointing quaternion
                q_desired = self._calculate_nadir_pointing(orbit_state)
            
            elif control_mode == 4:  # DOWNLOAD
                # Point +Z to Earth (same as nadir but inverted)
                q_nadir = self._calculate_nadir_pointing(orbit_state)
                r_nadir = Rotation.from_quat(q_nadir)
                r_download = r_nadir * Rotation.from_euler('x', 180, degrees=True)
                q_desired = r_download.as_quat()
            
            elif control_mode == 5:  # EOL
                # Point +Z along velocity vector for deorbit
                vel = np.array(orbit_state['velocity'])
                vel_unit = vel / np.linalg.norm(vel)
                rot = Rotation.align_vectors([[0, 0, 1]], [vel_unit])[0]
                q_desired = rot.as_quat()
            
            # Then update attitude based on current status
            if self.status == 2:  # POINTING ACHIEVED
                q_current, ang_rate = self._keep_pointing(q_desired)
                self.quaternion = q_current
                self.angular_rate = ang_rate
            else:  # SLEWING or other states
                q_current = self.quaternion
                q_current, ang_rate, errors = self._keep_slewing(q_desired, q_current)
                self.quaternion = q_current
                self.angular_rate = ang_rate

                # Update status based on pointing error
                error_angle = self._calculate_error_angle(q_current, q_desired)
                if error_angle > self.pointing_accuracy * 2.0:
                    self.status = 1  # SLEWING
                else:
                    if self.status != 2:  # Only log when first achieving pointing
                        met_sec = int((self.last_update_time - SIM_CONFIG['mission_start_time']).total_seconds())
                        self.logger.info(f"Pointing achieved in mode {self.MODES[self.mode]}")
                    self.status = 2  # POINTING ACHIEVED
                    # Add small random noise to rates when pointing
                    rand_noise = np.random.uniform(-0.001, 0.001, 3)
                    self.angular_rate = rand_noise

            self.angular_rate_total = np.linalg.norm(self.angular_rate)
            self.logger.debug(f"Updated attitude - quaternion: {self.quaternion}, angular_rate: {self.angular_rate}")
            
        except Exception as e:
            self.logger.error(f"Error in attitude update: {str(e)}")
            self.status = 0  # ERROR

    def _keep_pointing(self, q_desired):
        """Maintain pointing at desired quaternion with small corrections"""
        try:
            # Get current and desired rotations
            r_current = Rotation.from_quat(self.quaternion)
            r_desired = Rotation.from_quat(q_desired)
            
            # Calculate error rotation
            r_error = r_current.inv() * r_desired
            
            # Add small random noise to maintain stability
            noise_angle = np.random.uniform(-0.1, 0.1, 3)  # degrees
            noise_rot = Rotation.from_euler('xyz', noise_angle, degrees=True)
            
            # Apply noise to current attitude
            r_new = r_current * noise_rot
            
            # Calculate angular rates from noise
            ang_rate = noise_angle / self.time_step  # deg/s
            
            return r_new.as_quat(), ang_rate
            
        except Exception as e:
            self.logger.error(f"Error in keep_pointing: {str(e)}")
            return self.quaternion, np.zeros(3)

    def _keep_slewing(self, q_desired, q_current):
        """Update attitude during slewing maneuvers using SLERP"""
        try:
            # Get rotations from quaternions
            r_current = Rotation.from_quat(q_current)
            r_desired = Rotation.from_quat(q_desired)
            
            # Calculate error rotation
            r_error = r_current.inv() * r_desired
            error_angle = np.degrees(r_error.magnitude())  # Convert to degrees after getting magnitude
            
            # Calculate slew fraction based on nominal rate
            max_angle = self.nominal_slew_rate * self.time_step
            slew_fraction = min(max_angle / error_angle if error_angle > 0 else 1.0, 1.0)
            
            # Add some randomness to the slew rate
            slew_fraction *= np.random.uniform(0.9, 1.1)
            
            # Interpolate between current and desired attitude
            r_new = Rotation.from_quat(
                r_current.as_quat() * (1 - slew_fraction) + 
                r_desired.as_quat() * slew_fraction
            )
            
            # Calculate angular rates
            delta_angle = (r_new * r_current.inv()).as_euler('xyz', degrees=True)
            ang_rate = delta_angle / self.time_step
            
            # Limit angular rates
            ang_rate = np.clip(ang_rate, -self.max_slew_rate, self.max_slew_rate)
            
            # Get Euler angles for logging
            euler_current = r_current.as_euler('xyz', degrees=True)
            euler_desired = r_desired.as_euler('xyz', degrees=True)
            euler_error = euler_desired - euler_current
            
            # Debug logging
            modes = ['OFF', "DETUMBLE", "SUNPOINTING", "NADIR", "DOWNLOAD", "EOL"]
            self.logger.debug(f"Going to: {modes[self.mode]}")
            self.logger.debug(f"Error angle: {error_angle:.2f} deg")
            self.logger.debug(f"Slew fraction: {slew_fraction:.3f}")
            self.logger.debug(f"Angular rates: {ang_rate}")
            self.logger.debug(f"Euler errors: {euler_error}")
            
            return r_new.as_quat(), ang_rate, euler_error
            
        except Exception as e:
            self.logger.error(f"Error in keep_slewing: {str(e)}")
            return q_current, np.zeros(3), np.zeros(3)

    def _uncontrolled_motion(self):
        """Update attitude for uncontrolled spacecraft (OFF mode or UNCONTROLLED status)"""
        # Natural dynamics only - small random disturbances with damping
        MAX_RATE = 100.0  # Maximum rate in deg/s
        
        # Get current rates and apply random disturbances to get new rates
        new_rates = self.angular_rate + np.random.uniform(0.01, 0.03, 3)  # deg/s
        new_rates = np.clip(new_rates, -MAX_RATE, MAX_RATE)  # Limit maximum rates
        
        # Update quaternion based on rates
        dt = self.time_step
        delta_angles = new_rates * dt
        
        # Convert current quaternion to Euler angles
        roll, pitch, yaw = self._quaternion_to_euler(self.quaternion)
        
        # Update angles
        roll += delta_angles[0]
        pitch += delta_angles[1]
        yaw += delta_angles[2]
        
        # Convert back to quaternion
        self.quaternion = self._euler_to_quaternion(roll, pitch, yaw)
        self.angular_rate = new_rates
        self.angular_rate_total = np.linalg.norm(new_rates)

    def _detumble(self):
        """Update attitude for detumbling spacecraft (DETUMBLE mode)"""
        # Natural dynamics only - small random disturbances with damping
        DAMPING = 0.98  # Rate damping factor
        
        # Get current rates and apply damping and random disturbances
        new_rates = (self.angular_rate * DAMPING) + np.random.uniform(-0.1, 0.1, 3)
        
        # Update quaternion based on rates
        dt = self.time_step
        delta_angles = new_rates * dt
        
        # Convert current quaternion to Euler angles
        roll, pitch, yaw = self._quaternion_to_euler(self.quaternion)
        
        # Update angles
        roll += delta_angles[0]
        pitch += delta_angles[1]
        yaw += delta_angles[2]
        
        # Convert back to quaternion
        self.quaternion = self._euler_to_quaternion(roll, pitch, yaw)
        self.angular_rate = new_rates
        self.angular_rate_total = np.linalg.norm(new_rates)

    def _euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        """Convert Euler angles to quaternion using scipy.
        Returns quaternion in [x,y,z,w] format.
        """
        rot = Rotation.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True)
        return rot.as_quat()  # Returns [x,y,z,w] format

    def _quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles using scipy.
        Returns angles in degrees in xyz (roll, pitch, yaw) sequence.
        """
        rot = Rotation.from_quat(q)
        return rot.as_euler('xyz', degrees=True)

    def _calculate_error_angle(self, q_current, q_desired):
        """Calculate the angular error between current and desired quaternions using scipy"""
        try:
            r_current = Rotation.from_quat(q_current)
            r_desired = Rotation.from_quat(q_desired)
            # Calculate relative rotation and convert to angle
            r_error = r_current.inv() * r_desired
            return np.degrees(abs(r_error.magnitude()))  # Convert to degrees after getting magnitude
        except Exception as e:
            self.logger.error(f"Error calculating error angle: {str(e)}")
            return float('inf')  # Return large error on failure

    def _quaternion_multiply(self, q1, q2):
        """Multiply two quaternions using scipy"""
        r1 = Rotation.from_quat(q1)
        r2 = Rotation.from_quat(q2)
        return (r1 * r2).as_quat()

    def _rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion using scipy"""
        return Rotation.from_matrix(R).as_quat()

    @property
    def latitude(self):
        """Get current latitude in degrees"""
        return self.position[0]
        
    @property
    def longitude(self):
        """Get current longitude in degrees"""
        return self.position[1]
        
    @property
    def altitude(self):
        """Get current altitude in km"""
        return self.position[2]
    
    @property
    def burn_initiated(self):
        """Get burn initiation status"""
        return self._burn_initiated
    
    @burn_initiated.setter
    def burn_initiated(self, value):
        """Set burn initiation status"""
        self._burn_initiated = value

    @property
    def is_sunpointing(self):
        """Get current sunpointing status"""
        if self.mode == 1 and self.status == 2:
            return True
        else:
            return False
    
    def get_power_draw(self):
        """Get current power draw in Watts"""
        return self.power_draw

    def _calculate_sun_pointing(self, orbit_state, time):
        """Calculate sun pointing using orbit state"""
        try:
            # Get sun vector directly from orbit state
            sun_unit = orbit_state['sun_vector']
            
            # Create rotation that aligns body X axis with sun vector
            rot = Rotation.align_vectors([[1, 0, 0]], [sun_unit])[0]
            return rot.as_quat()
        except Exception as e:
            self.logger.error(f"Error calculating sun pointing: {str(e)}")
            return np.array([0, 0, 0, 1])

    def _calculate_nadir_pointing(self, orbit_state):
        """Calculate nadir pointing using orbit state"""
        try:
            # Get nadir vector directly from orbit state
            nadir = orbit_state['nadir_vector']
            
            # Create rotation that aligns body -Z axis with nadir
            rot = Rotation.align_vectors([[0, 0, -1]], [nadir])[0]
            return rot.as_quat()
        except Exception as e:
            self.logger.error(f"Error calculating nadir pointing: {str(e)}")
            return np.array([0, 0, 0, 1])

    def _calculate_velocity_pointing(self, orbit_state):
        """Calculate velocity pointing using orbit state"""
        try:
            # Get velocity vector directly from orbit state
            vel_unit = orbit_state['velocity_vector']
            
            # Create rotation that aligns body +Z with velocity
            rot = Rotation.align_vectors([[0, 0, 1]], [vel_unit])[0]
            return rot.as_quat()
        except Exception as e:
            self.logger.error(f"Error calculating velocity pointing: {str(e)}")
            return np.array([0, 0, 0, 1])

    def _calculate_angular_momentum(self):
        """Calculate spacecraft angular momentum in body frame"""
        try:
            # Approximate spacecraft as a uniform cube
            mass = 10.0 * u.kg  # Spacecraft mass
            size = 0.3 * u.m    # Cube side length
            
            # Calculate moments of inertia for a uniform cube
            I = mass * size**2 / 6.0
            I_tensor = np.array([
                [I.value, 0, 0],
                [0, I.value, 0],
                [0, 0, I.value]
            ])
            
            # Calculate angular momentum
            h = I_tensor @ self.angular_rate
            return h * u.kg * u.m**2 / u.s
            
        except Exception as e:
            self.logger.error(f"Error calculating angular momentum: {str(e)}")
            return np.zeros(3) * u.kg * u.m**2 / u.s
