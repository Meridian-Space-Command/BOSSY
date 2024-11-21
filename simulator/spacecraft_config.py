"""
Spacecraft Configuration File

This module defines the hardware configuration and initial conditions for the spacecraft
simulation. It is designed to be easily modified to match different spacecraft configurations.
Default configuration represents a 3U CubeSat with Earth observation capabilities.

References:
- CubeSat Design Specification Rev. 14
- CCSDS 133.0-B-2 Space Packet Protocol
"""

from datetime import datetime, timezone
import numpy as np

# Mission Parameters
MISSION_START_TIME = datetime(2025, 1, 1, 0, 0, 0, tzinfo=timezone.utc)

# Orbital Parameters (Default: 500km SSO)
ORBITAL_PARAMETERS = {
    'semi_major_axis': 6878.0,  # km (Earth radius + 500km)
    'eccentricity': 0.0,        # circular orbit
    'inclination': 97.4,        # degrees (sun-synchronous)
    'raan': 0.0,               # degrees
    'arg_perigee': 0.0,        # degrees
    'true_anomaly': 0.0,       # degrees
}

# Spacecraft Bus Configuration
SPACECRAFT_BUS = {
    'dimensions': {
        'x': 0.1,  # meters (3U CubeSat)
        'y': 0.1,  # meters
        'z': 0.3,  # meters
    },
    'mass': 4.0,   # kg
    'moment_of_inertia': {
        'Ixx': 0.033,  # kg⋅m²
        'Iyy': 0.033,  # kg⋅m²
        'Izz': 0.007,  # kg⋅m²
    }
}

# Power System
POWER_CONFIG = {
    'solar_panels': {
        'efficiency': 0.29,          # 29% efficient cells
        'area_per_panel': 0.03,      # m²
        'num_panels': 4,             # 4 deployable panels
    },
    'battery': {
        'capacity': 40.0,            # Wh
        'initial_charge': 1.0,       # 100%
        'voltage': 7.4,              # V
    },
    'power_consumption': {
        'obc': 0.5,                  # W
        'adcs': 1.5,                 # W
        'comms': 2.0,                # W (average)
        'payload': 4.0,              # W (when active)
    }
}

# ADCS Configuration
ADCS_CONFIG = {
    'magnetorquers': {
        'max_dipole': 0.2,           # Am²
        'num_rods': 3,               # one per axis
    },
    'reaction_wheels': {
        'max_torque': 0.001,         # Nm
        'max_momentum': 0.050,        # Nms
        'num_wheels': 3,             # one per axis
    },
    'sensors': {
        'magnetometer': True,
        'sun_sensors': True,
        'star_tracker': True,
        'gyroscope': True,
    }
}

# Payload Configuration (Earth Observation Camera)
PAYLOAD_CONFIG = {
    'camera': {
        'resolution': (2048, 2048),   # pixels
        'pixel_size': 5.5e-6,         # meters
        'focal_length': 0.070,        # meters
        'field_of_view': 15.0,        # degrees
        'spectral_bands': ['RGB'],
        'exposure_time': 0.001,       # seconds
    }
}

# Communications Configuration
COMMS_CONFIG = {
    'downlink': {
        'frequency': 437.5e6,         # Hz (UHF)
        'bandwidth': 9600,            # bps
        'tx_power': 1.0,              # W
    },
    'uplink': {
        'frequency': 145.9e6,         # Hz (VHF)
        'bandwidth': 1200,            # bps
    }
}

# Thermal Configuration
THERMAL_CONFIG = {
    'operating_temperature': {
        'min': -20,                   # °C
        'max': 50,                    # °C
    },
    'thermal_mass': 4.0,              # J/K
    'emissivity': 0.8,
    'absorptivity': 0.3,
}

# CCSDS Configuration
CCSDS_CONFIG = {
    'spacecraft_id': 0x42,
    'virtual_channel_id': 0x01,
    'apid_base': 0x100,              # Base APID for telemetry
    'packet_sequence_count': 0,
}