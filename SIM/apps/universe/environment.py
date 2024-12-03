from astropy import units as u
import numpy as np
from astropy.time import Time
from astropy.coordinates import get_sun
from astropy.coordinates.matrix_utilities import rotation_matrix
from config import UNIVERSE_CONFIG

class Environment:
    """Environmental models for space simulation"""
    
    def __init__(self):
        # Get radiation parameters from config
        self.rad_config = UNIVERSE_CONFIG['radiation']
        self.P_sr = self.rad_config['P_sr']
        self.R = self.rad_config['R']
        self.Cr = self.rad_config['Cr']
    
    @staticmethod
    def atmospheric_density(altitude):
        """Simple exponential atmospheric model
        
        Args:
            altitude: Altitude in km
            
        Returns:
            float: Density in kg/m³
        """
        h0 = 7.249 * u.km  # Scale height
        rho0 = 1.225 * u.kg / u.m**3  # Sea level density
        return (rho0 * np.exp(-altitude * u.km / h0)).to(u.kg / u.m**3).value
    
    def solar_illumination(self, orbit_state, sun, quaternion):
        """Calculate solar panel illumination angles using spacecraft state"""
        # Get spacecraft position
        r_sc = orbit_state['position'] * u.km
        
        # Get Sun position using astropy
        sun_gcrs = get_sun(Time(orbit_state['time']))
        r_sun = np.array([
            sun_gcrs.cartesian.x.to(u.km).value,
            sun_gcrs.cartesian.y.to(u.km).value,
            sun_gcrs.cartesian.z.to(u.km).value
        ]) * u.km
        
        # Calculate vectors
        sun_to_sc = r_sc - r_sun  # Vector from Sun to spacecraft
        sun_to_sc = sun_to_sc / np.linalg.norm(sun_to_sc)  # Normalize
        
        # Convert quaternion to rotation matrix
        q = quaternion
        R = self._quaternion_to_dcm(q)
        
        # Transform sun vector to body frame
        sun_body = R.T @ sun_to_sc.value
        
        # Calculate angles with radiation pressure effects
        angles = {}
        for axis, vec in [('pX', [1,0,0]), ('nX', [-1,0,0]), 
                         ('pY', [0,1,0]), ('nY', [0,-1,0])]:
            dot_product = np.dot(sun_body, vec)
            angle = np.arccos(np.clip(dot_product, -1, 1))
            angles[axis] = float(np.degrees(angle))
            # Apply radiation pressure coefficient
            angles[axis] *= self.Cr * self.R
            
        return angles

    def _quaternion_to_dcm(self, q):
        """Convert quaternion to Direction Cosine Matrix using astropy
        
        Args:
            q (np.array): Quaternion in [x,y,z,w] format
        Returns:
            np.array: 3x3 rotation matrix
        """
        x, y, z, w = q
        angle = 2 * np.arccos(w)
        axis = np.array([x, y, z]) / np.sin(angle/2) if angle != 0 else np.array([0, 0, 1])
        return rotation_matrix(angle, axis)
