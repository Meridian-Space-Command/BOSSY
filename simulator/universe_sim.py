"""
Universe Simulator

This module implements a basic universe simulator that models the Sun-Earth-Moon system
and related space physics phenomena. It provides the environmental context for spacecraft
operations, including orbital mechanics, atmospheric effects, radiation, and thermal conditions.

The simulator runs at 10Hz in realtime but can be accelerated using the speed multiplier
from the universe_config.
"""

import numpy as np
from datetime import datetime, timedelta
import logging
from typing import Dict, Tuple, Optional
from dataclasses import dataclass

from . import universe_config as cfg

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('spacecraft_simulator.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

@dataclass
class SpaceEnvironment:
    """Represents the space environment at a specific point and time"""
    position: np.ndarray          # [x, y, z] in ECI frame (m)
    velocity: np.ndarray          # [vx, vy, vz] in ECI frame (m/s)
    magnetic_field: np.ndarray    # [Bx, By, Bz] in ECI frame (Tesla)
    solar_flux: float            # W/m²
    atmospheric_density: float    # kg/m³
    is_eclipsed: bool            # True if in Earth's shadow
    temperature: float           # K
    radiation_flux: float        # particles/cm²/s

class UniverseSimulator:
    """Main universe simulation class"""
    
    def __init__(self, start_time: datetime = cfg.MISSION_START_TIME):
        """Initialize the universe simulator"""
        self.current_time = start_time
        self.dt = cfg.SIM_CONFIG['time_step']
        self.speed_multiplier = cfg.SIM_CONFIG['speed_multiplier']
        
        # Initialize celestial body positions
        self._update_celestial_positions()
        
        logger.info(f"Universe simulator initialized at {start_time}")

    def _update_celestial_positions(self) -> None:
        """Update positions of celestial bodies based on current time"""
        # Calculate Sun position in ECI frame
        self.sun_position = self._calculate_sun_position()
        
        # Calculate Moon position in ECI frame
        self.moon_position = self._calculate_moon_position()

    def _calculate_sun_position(self) -> np.ndarray:
        """Calculate Sun position in ECI frame"""
        # Simplified solar ephemeris calculation
        # In a production environment, use high-precision ephemeris like DE432s
        t = (self.current_time - datetime(2000, 1, 1, tzinfo=self.current_time.tzinfo)).days / 36525.0
        
        # Mean elements
        L0 = 280.46646 + 36000.76983 * t + 0.0003032 * t**2  # Mean longitude
        M = 357.52911 + 35999.05029 * t - 0.0001537 * t**2   # Mean anomaly
        e = 0.016708634 - 0.000042037 * t - 0.0000001267 * t**2  # Eccentricity
        
        # Convert to radians and calculate position
        L0 = np.radians(L0 % 360)
        M = np.radians(M % 360)
        
        # Calculate Sun position vector (simplified)
        r = cfg.SUN['mean_earth_distance'] * (1 - e**2) / (1 + e * np.cos(M))
        return r * np.array([np.cos(L0), np.sin(L0), 0])

    def _calculate_moon_position(self) -> np.ndarray:
        """Calculate Moon position in ECI frame"""
        # Simplified lunar ephemeris
        # In a production environment, use high-precision ephemeris
        t = (self.current_time - datetime(2000, 1, 1, tzinfo=self.current_time.tzinfo)).days / 36525.0
        
        # Mean elements
        L0 = 218.32 + 481267.883 * t  # Mean longitude
        M = 134.96 + 477198.867 * t   # Mean anomaly
        D = 297.85 + 445267.112 * t   # Mean elongation
        
        # Convert to radians
        L0 = np.radians(L0 % 360)
        M = np.radians(M % 360)
        D = np.radians(D % 360)
        
        # Calculate Moon position vector (simplified)
        r = cfg.MOON['mean_earth_distance']
        return r * np.array([np.cos(L0), np.sin(L0), np.sin(D)])

    def _calculate_magnetic_field(self, position: np.ndarray) -> np.ndarray:
        """Calculate magnetic field vector at given position"""
        # Simplified dipole model
        # In production, use IGRF model
        r = np.linalg.norm(position)
        
        # Safety check for zero position
        if r < 1e-10:
            return np.zeros(3)
        
        r_unit = position / r
        
        # Magnetic dipole moment (simplified)
        m = np.array([0, 0, 8.05e22])  # Earth's magnetic dipole moment
        B0 = 3.12e-5  # Tesla
        
        # Calculate magnetic field with safety checks
        B = (B0 / max(r**3, 1e-10)) * (3 * np.dot(r_unit, m) * r_unit - m)
        return B

    def _calculate_atmospheric_density(self, position: np.ndarray) -> float:
        """Calculate atmospheric density at given position"""
        r = np.linalg.norm(position)
        altitude = r - cfg.EARTH['radius']
        
        # Parameters for exponential atmosphere model
        rho0 = 1.225  # kg/m^3 (sea level)
        h = 7400  # m (scale height)
        
        # Limit altitude to avoid overflow
        max_altitude = 1000000  # 1000 km
        if altitude > max_altitude:
            return 0.0
        
        return rho0 * np.exp(-min(altitude, max_altitude) / h)

    def _check_eclipse(self, position: np.ndarray) -> bool:
        """Check if spacecraft is in Earth's shadow"""
        try:
            # Safety check for zero position
            r = np.linalg.norm(position)
            if r < 1e-10:
                return True
            
            pos_dir = position / r
            sun_dir = self.sun_position / np.linalg.norm(self.sun_position)
            angle = np.arccos(np.clip(np.dot(pos_dir, sun_dir), -1.0, 1.0))
            
            # Check if spacecraft is in Earth's shadow
            return angle < np.arcsin(np.clip(cfg.EARTH['radius'] / r, 0.0, 1.0))
        
        except Exception as e:
            logger.warning(f"Error in eclipse calculation: {e}")
            return False

    def get_environment_state(self, position: np.ndarray, velocity: np.ndarray) -> SpaceEnvironment:
        """Get complete space environment state for given position and velocity"""
        # Calculate altitude
        altitude = np.linalg.norm(position) - cfg.EARTH['radius'] * 1000  # in meters
        
        # Calculate all environmental parameters
        mag_field = self._calculate_magnetic_field(position)
        density = self._calculate_atmospheric_density(position / 1000)  # km to m
        eclipsed = self._check_eclipse(position)
        
        # Solar flux calculation
        base_flux = cfg.SUN['radiation_pressure']['solar_constant']
        solar_flux = 0.0 if eclipsed else base_flux * (cfg.SUN['mean_earth_distance'] / 
                                                      np.linalg.norm(self.sun_position))**2
        
        # Simple temperature model
        temperature = 250 if eclipsed else 300  # Simplified model
        
        # Basic radiation flux model
        radiation_flux = cfg.SPACE_ENVIRONMENT['cosmic_radiation']['galactic_flux']
        
        return SpaceEnvironment(
            position=position,
            velocity=velocity,
            magnetic_field=mag_field,
            solar_flux=solar_flux,
            atmospheric_density=density,
            is_eclipsed=eclipsed,
            temperature=temperature,
            radiation_flux=radiation_flux
        )

    def step(self) -> None:
        """Advance simulation by one time step"""
        self.current_time += timedelta(seconds=self.dt * self.speed_multiplier)
        self._update_celestial_positions()

    @property
    def time(self) -> datetime:
        """Current simulation time"""
        return self.current_time

    def set_speed_multiplier(self, multiplier: float) -> None:
        """Set simulation speed multiplier"""
        if multiplier <= 0:
            raise ValueError("Speed multiplier must be positive")
        self.speed_multiplier = multiplier
        logger.info(f"Simulation speed set to {multiplier}x realtime")

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Return the conjugate of a quaternion"""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector by a quaternion"""
    qv = np.array([0, v[0], v[1], v[2]])
    q_conj = quaternion_conjugate(q)
    return quaternion_multiply(quaternion_multiply(q, qv), q_conj)[1:]