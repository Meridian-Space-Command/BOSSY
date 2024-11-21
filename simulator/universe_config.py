"""
Universe Configuration File

This module defines the parameters for the universe simulation, including celestial bodies,
space environment models, and simulation settings. It provides a basic but physically accurate
model of the Sun-Earth-Moon system and related space physics phenomena.

References:
- NASA Planetary Fact Sheets
- International Geomagnetic Reference Field (IGRF-13)
- NRLMSISE-00 Atmosphere Model
- Space Environment Standards
"""

from datetime import datetime, timezone
import numpy as np
from .spacecraft_config import MISSION_START_TIME

# Simulation Control
SIM_CONFIG = {
    'time_step': 1.0,          # seconds (1 Hz)
    'speed_multiplier': 1.0,   # 1x real-time (can be adjusted: 1, 5, 20, etc.)
}

# Celestial Bodies
EARTH = {
    'mass': 5.972e24,          # kg
    'radius': 6371.0,          # km
    'J2': 1.082635854e-3,      # Earth's J2 gravitational coefficient
    'rotation_period': 86164.1, # seconds (sidereal day)
    'mu': 3.986004418e14,      # m³/s² (gravitational parameter)
    'atmosphere': {
        'max_altitude': 1000.0, # km (for atmospheric effects)
        'base_density': 1.225,  # kg/m³ (at sea level)
        'scale_height': 7.249,  # km (approximate for lower thermosphere)
    },
    'magnetic_field': {
        'dipole_strength': 7.94e22,  # A⋅m²
        'dipole_tilt': 11.0,         # degrees
        'update_period': 3600,       # seconds (update IGRF model)
    }
}

SUN = {
    'mass': 1.989e30,          # kg
    'radius': 696340.0,        # km
    'luminosity': 3.828e26,    # W
    'mean_earth_distance': 149597870.7,  # km
    'radiation_pressure': {
        'solar_constant': 1361.0,    # W/m² at 1 AU
        'pressure_1au': 4.56e-6,     # N/m² at 1 AU
    }
}

MOON = {
    'mass': 7.34767309e22,     # kg
    'radius': 1737.1,          # km
    'mu': 4.9048695e12,        # m³/s² (gravitational parameter)
    'mean_earth_distance': 384400.0,  # km
    'orbital_period': 27.321661,      # days
    'orbital_elements': {
        'semi_major_axis': 384400.0,  # km
        'eccentricity': 0.0549,
        'inclination': 5.145,         # degrees
    }
}

# Space Environment
SPACE_ENVIRONMENT = {
    'radiation_belts': {
        'inner_belt_range': (1000, 6000),     # km altitude
        'outer_belt_range': (13000, 40000),   # km altitude
        'max_flux': 1e8,                      # particles/cm²/s
    },
    'solar_activity': {
        'default_f10_7': 150.0,               # Solar radio flux (10.7 cm)
        'default_ap': 15,                     # Geomagnetic Ap index
        'solar_cycle_length': 11.0,           # years
    },
    'cosmic_radiation': {
        'galactic_flux': 0.4,                 # particles/cm²/s
        'solar_particle_events': False,        # Enable/disable SPE simulation
    }
}

# Orbital Perturbations
PERTURBATIONS = {
    'enabled': {
        'J2': True,                    # Earth's oblateness
        'atmospheric_drag': True,
        'solar_radiation_pressure': True,
        'third_body': {
            'sun': True,
            'moon': True
        },
        'magnetic_torque': True
    },
    'minimum_force': 1e-12             # N (forces below this are ignored)
}

# Time and Reference Frames
REFERENCE_FRAMES = {
    'primary': 'J2000',                # Inertial reference frame
    'supported': [
        'J2000',                       # Earth-centered inertial
        'ECEF',                        # Earth-centered Earth-fixed
        'LVLH',                        # Local vertical, local horizontal
    ],
    'precession_nutation': True,       # Enable precession/nutation calculations
    'polar_motion': True,              # Enable polar motion corrections
}

# Physical Constants
CONSTANTS = {
    'G': 6.67430e-11,                 # m³/kg/s² (gravitational constant)
    'c': 299792458.0,                 # m/s (speed of light)
    'k': 1.380649e-23,                # J/K (Boltzmann constant)
    'stefan_boltzmann': 5.670374419e-8,  # W/m²/K⁴
}