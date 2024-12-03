import numpy as np
from config import ORBIT_CONFIG, SIM_CONFIG, UNIVERSE_CONFIG
from astropy import units as u
from poliastro.bodies import Earth, Sun, Moon  # Import the classes directly
from poliastro.twobody import Orbit
from poliastro.ephem import Ephem
from poliastro.core.perturbations import J2_perturbation, atmospheric_drag_exponential
from poliastro.twobody.propagation import CowellPropagator
from poliastro.core.propagation import func_twobody
from poliastro.core.elements import rv2coe
from poliastro.util import time_range
from astropy.time import Time
from astropy.coordinates import GCRS, ITRS, CartesianRepresentation, SphericalRepresentation, get_sun
from logger import SimLogger  # Import SimLogger

class SimulationEndedException(Exception):
    """Exception raised when simulation should end"""
    pass

class OrbitPropagator:
    def __init__(self):
        # Initialize logger
        self.logger = SimLogger.get_logger('SimLogger')
        
        # Initialize celestial bodies with mission start time
        self.mission_start = SIM_CONFIG['mission_start_time']
        self.earth = Earth  # Use the class directly
        self.sun = Sun
        self.moon = Moon
        self._in_eclipse = False
        self._eclipse_type = None
        
        # Get initial orbital elements from config
        orbit_elements = ORBIT_CONFIG['spacecraft']['elements']

        self.a = orbit_elements['semi_major_axis'] << u.km
        self.ecc = orbit_elements['eccentricity'] << u.one
        self.inc = orbit_elements['inclination'] << u.deg
        self.raan = orbit_elements['raan'] << u.deg
        self.argp = orbit_elements['arg_perigee'] << u.deg
        self.nu = orbit_elements['true_anomaly'] << u.deg

        # Initialize orbit with perturbations based on config
        self.perturbations = []
        
        if UNIVERSE_CONFIG['perturbations']['J2']:
            # J2 perturbation requires R and J2 value
            R = self.earth.R.to(u.km)
            J2 = self.earth.J2.value
            self.perturbations.append(
                lambda t0, state, k: J2_perturbation(
                    t0, state, k,
                    J2=J2,
                    R=R
                )
            )
            self.logger.info("J2 perturbation enabled")
            
        if UNIVERSE_CONFIG['perturbations']['atmospheric']:
            # Get atmosphere parameters from config
            atmos_config = UNIVERSE_CONFIG['atmosphere']
            
            # Convert parameters to proper units
            R = self.earth.R.to(u.km)
            C_D = 2.2 * u.dimensionless_unscaled  # Typical drag coefficient
            A_over_m = (0.1 * u.m**2 / (10.0 * u.kg)).to(u.km**2 / u.kg)  # Area/mass ratio
            H0 = 7.249 * u.km  # Scale height
            rho0 = 1.225 * u.kg / u.m**3  # Sea level density
            
            self.perturbations.append(
                lambda t0, state, k: atmospheric_drag_exponential(
                    t0, state, k,
                    R=R,
                    C_D=C_D,
                    A_over_m=A_over_m,
                    H0=H0,
                    rho0=rho0
                )
            )
            self.logger.info("Atmospheric drag enabled")
            
        if UNIVERSE_CONFIG['perturbations']['radiation']:
            # Solar radiation pressure could be added here when supported by poliastro
            self.logger.warning("Solar radiation pressure not yet implemented")

        # Create orbit object with perturbations
        orb = Orbit.from_classical(
            self.earth, 
            self.a, self.ecc, self.inc, 
            self.raan, self.argp, self.nu,
            epoch=Time(self.mission_start)
        )

        # Calculate orbital period
        self.period = orb.period.to(u.s)
        self.last_update_time = Time(self.mission_start)
        self._current_state = orb

        # Calculate future orbit points (one full orbit)
        future_points = []
        period_seconds = self.period.to(u.s).value
        num_points = int(period_seconds / 30)  # Points every 30 seconds
        
        for i in range(num_points):
            future_time = Time(self.mission_start) + (i * 30 * u.s)
            future_state = self._current_state.propagate(future_time)
            
            # Convert position to lat/lon
            pos = future_state.r
            gcrs = GCRS(CartesianRepresentation(x=pos[0], y=pos[1], z=pos[2]), 
                        obstime=future_time)
            itrs = gcrs.transform_to(ITRS(obstime=future_time))
            location = itrs.represent_as(SphericalRepresentation)
            
            lat = location.lat.to(u.deg).value
            lon = location.lon.wrap_at(180 * u.deg).to(u.deg).value
            future_points.append([lat, lon])
        
        self.future_orbit = future_points

        self.logger.info(f"Orbit initialized at epoch {self.mission_start}")
        self.logger.info(
            f"Initial orbital elements: "
            f"semi_major_axis={self._current_state.a.to(u.km):.1f}, "
            f"eccentricity={self._current_state.ecc:.4f}, "
            f"inclination={self._current_state.inc.to(u.deg):.1f}°, "
            f"raan={self._current_state.raan.to(u.deg):.1f}°, "
            f"arg_perigee={self._current_state.argp.to(u.deg):.1f}°, "
            f"true_anomaly={self._current_state.nu.to(u.deg):.1f}°"
        )

    def propagate(self, timestamp):
        """Propagate orbit and return complete state"""
        self.last_update_time = Time(timestamp)
        
        # Combine perturbations into a single function
        def f(t0, u_, k):
            du_kep = func_twobody(t0, u_, k)
            
            # Sum all perturbation accelerations
            ax = ay = az = 0
            if self.perturbations:
                for perturbation in self.perturbations:
                    dax, day, daz = perturbation(t0, u_, k)
                    ax += dax
                    ay += day
                    az += daz
            
            du_ad = np.array([0, 0, 0, ax, ay, az])
            return du_kep + du_ad

        # Propagate orbit with perturbations if enabled
        if self.perturbations:
            self._current_state = self._current_state.propagate(
                self.last_update_time,
                method=CowellPropagator(f=f)
            )
        else:
            self._current_state = self._current_state.propagate(self.last_update_time)
        
        # Calculate future orbit points (one full orbit)
        future_points = []
        period_seconds = self.period.to(u.s).value
        num_points = int(period_seconds / 30)  # Points every 30 seconds
        
        for i in range(num_points):
            future_time = Time(timestamp) + (i * 30 * u.s)
            future_state = self._current_state.propagate(future_time)
            
            # Convert position to lat/lon
            pos = future_state.r
            gcrs = GCRS(CartesianRepresentation(x=pos[0], y=pos[1], z=pos[2]), 
                        obstime=future_time)
            itrs = gcrs.transform_to(ITRS(obstime=future_time))
            location = itrs.represent_as(SphericalRepresentation)
            
            lat = location.lat.to(u.deg).value
            lon = location.lon.wrap_at(180 * u.deg).to(u.deg).value
            future_points.append([lat, lon])
        
        # Get current position and velocity in Earth-centered inertial frame
        pos = self._current_state.r
        vel = self._current_state.v
        
        # Convert position to lat/lon using astropy coordinates
        gcrs = GCRS(CartesianRepresentation(x=pos[0], y=pos[1], z=pos[2]), 
                    obstime=self.last_update_time)
        itrs = gcrs.transform_to(ITRS(obstime=self.last_update_time))
        location = itrs.represent_as(SphericalRepresentation)
        
        # Get lat/lon with proper wrapping
        lat = location.lat.to(u.deg).value
        lon = location.lon.wrap_at(180 * u.deg).to(u.deg).value
        
        # Track latitude changes to determine direction
        if not hasattr(self, '_last_lat'):
            self._last_lat = lat
            self._crossing_direction = None
        
        # Near equator, determine crossing direction
        if abs(lat) < 1.0:  # Near equator
            if self._last_lat > lat:
                self._crossing_direction = 'south'  # Heading south
            elif self._last_lat < lat:
                self._crossing_direction = 'north'  # Heading north
        elif abs(lat) > 1.0:  # Away from equator
            self._crossing_direction = None
        
        # Force continuation of direction through equator
        if self._crossing_direction == 'south' and lat > self._last_lat:
            lat = -abs(lat)  # Force into southern hemisphere
        elif self._crossing_direction == 'north' and lat < self._last_lat:
            lat = abs(lat)   # Force into northern hemisphere
        
        self._last_lat = lat

        # Calculate eclipse condition using position vectors
        # Get Sun position using astropy
        sun_gcrs = get_sun(self.last_update_time)
        # Convert from AU to km
        r_sun = np.array([
            sun_gcrs.cartesian.x.to(u.km).value,
            sun_gcrs.cartesian.y.to(u.km).value,
            sun_gcrs.cartesian.z.to(u.km).value
        ]) * u.km
        
        # Calculate vectors
        sun_to_sc = pos - r_sun  # Vector from Sun to spacecraft
        r_earth = -pos  # Vector from spacecraft to Earth center
        
        # Calculate angles and distances for eclipse geometry
        sun_angle = np.arccos(np.dot(-sun_to_sc, r_earth) / 
                             (np.linalg.norm(sun_to_sc) * np.linalg.norm(r_earth)))
        earth_angular_radius = np.arcsin(self.earth.R / np.linalg.norm(r_earth))
        sun_angular_radius = np.arcsin(self.sun.R / np.linalg.norm(sun_to_sc))
        
        # Determine eclipse condition
        if sun_angle < earth_angular_radius - sun_angular_radius:
            self._in_eclipse = True
            self._eclipse_type = 'umbra'
        elif sun_angle < earth_angular_radius + sun_angular_radius:
            self._in_eclipse = True
            self._eclipse_type = 'penumbra'
        else:
            self._in_eclipse = False
            self._eclipse_type = None
            
        self.logger.debug(f"Eclipse state: {self._eclipse_type}")
        
        # Get current orbital elements
        alt = (np.linalg.norm(pos) - self.earth.R).to(u.km).value  # Height above Earth's surface
        
        # Get sun vector in GCRS
        sun_gcrs = get_sun(Time(timestamp))
        sun_vector = sun_gcrs.cartesian.xyz.value * u.au.to(u.km)
        
        # Calculate additional state vectors
        pos_unit = pos / np.linalg.norm(pos)
        vel_unit = vel / np.linalg.norm(vel)
        h_vector = np.cross(pos.value, vel.value) * u.km**2/u.s  # Angular momentum vector
        
        # Create complete state dictionary
        state = {
            'position': pos.to(u.km).value,  # [x, y, z] in km
            'velocity': vel.to(u.km/u.s).value,  # [vx, vy, vz] in km/s
            'sun_vector': sun_vector,  # Unit vector to Sun
            'nadir_vector': -pos_unit.value,  # Unit vector to Earth center
            'velocity_vector': vel_unit.value,  # Unit vector in velocity direction
            'h_vector': h_vector.value,  # Orbital angular momentum vector
            'lat': lat,
            'lon': lon,
            'alt': alt,  # Height above Earth's surface in km
            'true_anomaly': self._current_state.nu.to(u.deg).value % 360,
            'eclipse': self.eclipse_status,
            'time': timestamp,
            # Orbital elements
            'semi_major_axis': self._current_state.a.to(u.km).value,
            'eccentricity': self._current_state.ecc.value,
            'inclination': self._current_state.inc.to(u.deg).value,
            'raan': self._current_state.raan.to(u.deg).value,
            'arg_perigee': self._current_state.argp.to(u.deg).value,
            'future_path': self.future_orbit,  # Add pre-calculated future points
        }
        
        # Check for reentry
        if state['alt'] < 50 + np.random.uniform(-7, 7):
            self.logger.debug("Altitude is less than 50km - stopping simulation")
            raise SimulationEndedException("Spacecraft LOST. This is the end my friend.")
        
        return state
    
    def burn(self):
        """Initiate deorbit burn by reducing semi-major axis"""
        self.logger.info("Initiating deorbit burn")
        
        # Calculate change in semi-major axis for 4km altitude decrease
        delta_a = -4 * u.km
        
        # Create new orbit with updated semi-major axis
        new_orbit = Orbit.from_classical(
            self.earth,
            self._current_state.a + delta_a,
            self._current_state.ecc,
            self._current_state.inc,
            self._current_state.raan,
            self._current_state.argp,
            self._current_state.nu,
            epoch=self.last_update_time
        )
        
        self._current_state = new_orbit
        self.period = new_orbit.period
        
        self.logger.debug(f"New semi-major axis: {new_orbit.a.to(u.km):.1f}")
    
    @property
    def current_state(self):
        return self._current_state
    
    @current_state.setter
    def current_state(self, value):
        self._current_state = value
    
    @property
    def in_eclipse(self):
        return self._in_eclipse
    
    @in_eclipse.setter
    def in_eclipse(self, value):
        self._in_eclipse = value
    
    @property 
    def eclipse_status(self):
        """Return current eclipse status"""
        return {
            'in_eclipse': bool(self._in_eclipse),  # Ensure it's a bool
            'type': self._eclipse_type
        }

    def _calculate_frames(self, pos, vel, time):
        """Calculate all reference frames and transformations"""
        try:
            # GCRS (Earth-centered inertial) frame
            gcrs = GCRS(CartesianRepresentation(x=pos[0], y=pos[1], z=pos[2]), 
                        obstime=time)
            
            # ITRS (Earth-fixed) frame
            itrs = gcrs.transform_to(ITRS(obstime=time))
            
            # TEME (True Equator Mean Equinox) frame for velocity
            teme_pos = pos.to(u.m)
            teme_vel = vel.to(u.m/u.s)
            
            return {
                'gcrs': gcrs,
                'itrs': itrs,
                'teme_pos': teme_pos,
                'teme_vel': teme_vel
            }
        except Exception as e:
            self.logger.error(f"Error calculating reference frames: {e}")
            return None

    def _calculate_eclipse(self, pos, sun_pos):
        """Calculate eclipse state using astropy"""
        try:
            # Get vectors
            r_sc = pos
            r_sun = sun_pos.get_gcrs(self.last_update_time).cartesian
            
            # Calculate eclipse geometry
            sun_to_sc = r_sc - r_sun
            earth_to_sc = r_sc
            
            # Use astropy's built-in angle calculation
            sun_angle = sun_to_sc.separation(earth_to_sc)
            earth_angle = np.arcsin(self.earth.R / np.linalg.norm(earth_to_sc))
            sun_angle = np.arcsin(self.sun.R / np.linalg.norm(sun_to_sc))
            
            return self._determine_eclipse_state(sun_angle, earth_angle)
        except Exception as e:
            self.logger.error(f"Error calculating eclipse: {e}")
            return {'in_eclipse': False, 'type': None}

    def _calculate_state_vectors(self, pos, vel):
        """Calculate state vectors in various frames"""
        try:
            # Unit vectors
            pos_unit = pos / np.linalg.norm(pos)
            vel_unit = vel / np.linalg.norm(vel)
            
            # Angular momentum
            h_vec = np.cross(pos.value, vel.value) * (u.km**2/u.s)
            h_unit = h_vec / np.linalg.norm(h_vec)
            
            # Orbital frame vectors
            radial = pos_unit
            normal = h_unit
            transverse = np.cross(normal.value, radial.value)
            
            return {
                'position_unit': pos_unit.value,
                'velocity_unit': vel_unit.value,
                'h_vector': h_vec.value,
                'radial': radial.value,
                'normal': normal.value,
                'transverse': transverse
            }
        except Exception as e:
            self.logger.error(f"Error calculating state vectors: {e}")
            return None
