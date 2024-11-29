import numpy as np
from .bodies import Earth, Sun, Moon
from config import ORBIT_CONFIG, SIM_CONFIG

class SimulationEndedException(Exception):
    """Exception raised when simulation should end"""
    pass

class OrbitPropagator:
    def __init__(self):
        # Initialize logger
        import logging
        self.logger = logging.getLogger('SimLogger')
        
        # Initialize celestial bodies with mission start time
        self.mission_start = SIM_CONFIG['mission_start_time']
        self.earth = Earth()
        self.sun = Sun()
        self.moon = Moon()
        
        # Get initial orbital elements from config
        orbit_elements = ORBIT_CONFIG['spacecraft']['elements']
        
        # only set semi-major axis if not already set
        if not hasattr(self, 'a'):
            self.a = orbit_elements['semi_major_axis'] * 1000  # km to m
        self.e = orbit_elements['eccentricity']
        self.i = np.radians(orbit_elements['inclination'])
        self.raan = np.radians(orbit_elements['raan'])
        self.arg_p = np.radians(orbit_elements['arg_perigee'])
        self.true_anomaly = np.radians(orbit_elements['true_anomaly'])

        # Calculate orbital period
        self.period = 2 * np.pi * np.sqrt(self.a**3 / self.earth.gravitational_parameter())
        
        # Calculate initial state
        self.last_update_time = self.mission_start
        self.current_state = self._calculate_state(self.mission_start)
        
        self.logger.info(f"Orbit initialized at epoch {self.mission_start}")
        self.logger.info(f"Initial orbital elements: a={self.a/1000:.1f}km, e={self.e:.4f}, i={np.degrees(self.i):.1f}°")
        
    def _calculate_state(self, time):
        """Calculate orbital state at given time"""
        dt = (time - self.mission_start).total_seconds()

        # Calculate mean motion (n)
        n = np.sqrt(self.earth.gravitational_parameter() / self.a**3)  # More accurate than 2π/period
        
        # Convert initial true anomaly to initial mean anomaly
        E0 = 2 * np.arctan(np.sqrt((1 - self.e)/(1 + self.e)) * np.tan(self.true_anomaly/2))
        M0 = E0 - self.e * np.sin(E0)
        
        # Calculate current mean anomaly
        M = (M0 + n * dt) % (2 * np.pi)
        
        # Solve Kepler's equation using Newton-Raphson method (more accurate)
        E = M  # Initial guess
        for _ in range(10):
            f = E - self.e * np.sin(E) - M
            f_prime = 1 - self.e * np.cos(E)
            E_next = E - f/f_prime
            if abs(E_next - E) < 1e-8:  # Convergence check
                E = E_next
                break
            E = E_next
        
        # Calculate true anomaly using a more numerically stable formula
        nu = 2 * np.arctan2(
            np.sqrt(1 + self.e) * np.sin(E/2),
            np.sqrt(1 - self.e) * np.cos(E/2)
        )
        
        # Calculate radius vector magnitude
        r = self.a * (1 - self.e * np.cos(E))
        
        # Calculate position in orbital plane (perifocal coordinates)
        x = r * np.cos(nu)
        y = r * np.sin(nu)
        pos_orbital = np.array([x, y, 0])
        
        # Create rotation matrix from perifocal to ECI
        # Note: Order is important! RAAN -> inclination -> arg of perigee
        R_W = np.array([
            [np.cos(self.raan), -np.sin(self.raan), 0],
            [np.sin(self.raan), np.cos(self.raan), 0],
            [0, 0, 1]
        ])
        
        R_i = np.array([
            [1, 0, 0],
            [0, np.cos(self.i), -np.sin(self.i)],
            [0, np.sin(self.i), np.cos(self.i)]
        ])
        
        R_w = np.array([
            [np.cos(self.arg_p), -np.sin(self.arg_p), 0],
            [np.sin(self.arg_p), np.cos(self.arg_p), 0],
            [0, 0, 1]
        ])
        
        # Transform to ECI (apply rotations in correct order)
        R_pef_to_eci = R_W @ R_i @ R_w
        pos_eci = R_pef_to_eci @ pos_orbital
        
        # Calculate velocity in orbital plane
        p = self.a * (1 - self.e**2)  # Semi-latus rectum
        mu = self.earth.gravitational_parameter()
        
        # Velocity components in perifocal frame
        v_x = -np.sqrt(mu/p) * np.sin(nu)
        v_y = np.sqrt(mu/p) * (self.e + np.cos(nu))
        vel_orbital = np.array([v_x, v_y, 0])
        
        # Transform velocity to ECI using same rotation matrix
        vel_eci = R_pef_to_eci @ vel_orbital
        
        # Apply J2 perturbation before calculating lat/lon
        j2_accel = self._apply_j2_perturbation(pos_eci, dt)
        pos_eci += 0.5 * j2_accel * dt**2  # Simple numerical integration
        
        # Calculate lat, lon, alt from perturbed position
        r_mag = np.sqrt(np.sum(pos_eci**2))
        lat = np.arcsin(pos_eci[2] / r_mag)
        lon = np.arctan2(pos_eci[1], pos_eci[0])
        alt = r_mag - self.earth.radius
        
        # Check eclipse condition
        sun_pos = self.sun.get_position_at_time(time)
        in_eclipse = self._check_eclipse(pos_eci, sun_pos)
        
        # Create the state dictionary
        state = {
            'position': pos_eci / 1000,  # m to km
            'velocity': vel_eci / 1000,  # m/s to km/s
            'lat': np.degrees(lat),
            'lon': np.degrees(lon),
            'alt': alt / 1000,  # m to km
            'eclipse': in_eclipse,
            'time': time,
            'true_anomaly': np.degrees(nu) % 360  # Added for debugging
        }
        
        # Check for reentry
        if state['alt'] < 50 + np.random.uniform(-7, 7):
            self.logger.debug("Altitude is less than 50km - stopping simulation")
            raise SimulationEndedException("Spacecraft LOST. This is the end my friend.")
        
        return state
        
    def propagate(self, current_time):
        """Update and return orbital state"""
        if current_time != self.last_update_time:
            self.current_state = self._calculate_state(current_time)
            self.last_update_time = current_time
            self.logger.debug(f"Orbit updated to {current_time}: lat={self.current_state['lat']:.1f}°, " +
                            f"lon={self.current_state['lon']:.1f}°, alt={self.current_state['alt']:.1f}km")
        
        return self.current_state
    
    def burn(self):
        """Initiate deorbit burn by reducing semi-major axis"""
        self.logger.info("Initiating deorbit burn")
        
        # Calculate change in semi-major axis for 4km altitude decrease
        self.a -= 4000  # 4km in meters
        
        # Recalculate orbital period with new semi-major axis
        self.period = 2 * np.pi * np.sqrt(self.a**3 / self.earth.gravitational_parameter())
        
        self.logger.debug(f"New semi-major axis: {self.a/1000:.1f}km")
        
    def _check_eclipse(self, pos_eci, sun_pos):
        """Simple cylindrical shadow model"""
        sun_dir = sun_pos / np.linalg.norm(sun_pos)
        sc_dir = pos_eci / np.linalg.norm(pos_eci)
        
        angle = np.arccos(np.dot(sun_dir, sc_dir))
        return angle > np.pi/2
        
    def _apply_j2_perturbation(self, pos_eci, dt):
        """Apply J2 perturbation to orbital elements
        Input: position in ECI [m], time step [s]
        Returns: perturbation accelerations [m/s²]
        """
        # J2 perturbation calculation
        x, y, z = pos_eci
        r = np.sqrt(np.sum(pos_eci**2))
        
        # Pre-calculate terms
        j2_term = -1.5 * self.earth.J2 * self.earth.gravitational_parameter() * self.earth.radius**2 / r**4
        
        # Calculate accelerations
        ax = j2_term * x/r * (5*z**2/r**2 - 1)
        ay = j2_term * y/r * (5*z**2/r**2 - 1)
        az = j2_term * z/r * (5*z**2/r**2 - 3)
        
        return np.array([ax, ay, az])
