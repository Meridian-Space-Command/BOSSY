from datetime import datetime

LOGGER_CONFIG = {
    'level': 'DEBUG',  # Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
}

# Simulator Configuration
SIM_CONFIG = {
    'epoch': datetime(2000, 1, 1, 0, 0, 0),  # Epoch time (don't change this!!!)
    'mission_start_time': datetime(2025, 1, 1, 0, 0, 0),  # Mission start time
    'time_step': 1.0,                                     # Simulation time step in seconds
    'download_directory': './BOSSY/MCS/download/',
    'upload_directory': './BOSSY/MCS/upload/',
    'google_maps_api_key': 'AIzaSyDL__brVoZ4VY72_ZnRl5MhLnWLpuP4bsA'
}

# Spacecraft Configuration
SPACECRAFT_CONFIG = {
    'spacecraft': {
        'comms': {
            'host': 'localhost',
            'tc_port': 10025,  # Telecommand reception port
            'tm_port': 10015   # Telemetry transmission port
        },
        'initial_state': {
            # OBC Initial State
            'obc': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'power_draw': 0.6,    # float_W: Power consumption in Watts
                'mode': 2,            # OBCMode_Type (uint8): 0=SAFE, 1=NOMINAL, 2=ALLON
                'uptime': 0           # uint32_s: Uptime in seconds
            },

            # CDH Initial State
            'cdh': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'power_draw': 0.4,    # float_W: Power consumption in Watts
            },

            # POWER Initial State
            'power': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'power_draw': 0.4,    # float_W: Power consumption in Watts
                'battery_voltage': 7.95,# float_V: Battery voltage in Volts
                'battery_current': 0.0,# float_A: Battery current in Amperes
                'power_balance': 0,   # PowerBalance_Type (uint8): 0=BALANCED, 1=POSITIVE, 2=NEGATIVE
                'solar_total_generation': 0.0,  # float_W: Total solar power generation in Watts
                'solar_panel_generation': {     # float_W: Per-panel solar generation in Watts
                    'pX': 0.0,       # +X panel power generation
                    'nX': 0.0,       # -X panel power generation
                    'pY': 0.0,       # +Y panel power generation
                    'nY': 0.0        # -Y panel power generation
                }
            },

            # ADCS Initial State
            'adcs': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'power_draw': 2.3,    # float_W: Power consumption in Watts
                'mode': 0,           # ADCSMode_Type (uint8): 0=UNCONTROLLED, 1=LOCK, 2=SUNPOINTING, 3=NADIR, 4=DOWNLOAD
                'status': 0,         # ADCSStatus_Type (uint8): 0=UNCONTROLLED, 1=SLEWING, 2=POINTING
                'quaternion': [0.707, 0.0, 0.0, 0.707],  # float[4]: Attitude quaternion [q1,q2,q3,q4]
                'angular_rate': [0.0, 0.0, 0.0],     # float_deg_s[3]: Angular rates [x,y,z] in deg/s
                'position': [0.0, 0.0, 500.0],           # [float_deg,float_deg,float_km]: [lat,lon,alt]
                'eclipse': 0         # Eclipse_Type (uint8): 0=DAY, 1=NIGHT
            },

            # COMMS Initial State
            'comms': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'rx_power_draw': 1.0, # float_W: Power consumption in Watts
                'txrx_power_draw': 3.7,    # float_W: Power consumption in Watts
                'mode': 1,           # CommsMode_Type (uint8): 0=RX, 1=TXRX
                'uplink_bitrate': 32000,   # uint32_bps: Uplink data rate in bits per second
                'downlink_bitrate': 128000  # uint32_bps: Downlink data rate in bits per second
            },

            # PAYLOAD Initial State
            'payload': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'idle_power_draw': 0.5, # float_W: Power consumption in Watts when idle
                'capturing_power_draw': 4.7,    # float_W: Power consumption in Watts when capturing
                'status': 0          # PayloadStatus_Type (uint8): 0=IDLE, 1=CAPTURING
            },

            # DATASTORE Initial State
            'datastore': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'power_draw': 0.4,    # float_W: Power consumption in Watts
                'storage_path': "./apps/spacecraft/storage/", # Path to the storage directory, relative to the BOSSY/SIM/ directory
                'storage_total': 1024 * 1024 * 1024,  # uint32: Total storage in bytes (1GB)
                'mode': 0           # DatastoreMode_Type (uint8): 0=IDLE, 1=DOWNLOADING, 2=UPLOADING
            }
        },
        'hardware': {
            'adcs': {
                'magnetorquers': {
                    'x': {'max_moment': 0.2},     # A⋅m² (magnetic dipole moment)
                    'y': {'max_moment': 0.2},
                    'z': {'max_moment': 0.2}
                },
                'reaction_wheels': {
                    'x': {
                        'max_torque': 0.001,      # N⋅m
                        'max_momentum': 0.05,      # N⋅m⋅s
                        'moment_of_inertia': 0.001 # kg⋅m²
                    },
                    'y': {
                        'max_torque': 0.001,
                        'max_momentum': 0.05,
                        'moment_of_inertia': 0.001
                    },
                    'z': {
                        'max_torque': 0.001,
                        'max_momentum': 0.05,
                        'moment_of_inertia': 0.001
                    }
                },
                'pointing_requirements': {
                    'accuracy_threshold': 2.0,     # degrees, threshold between SLEWING and POINTING
                    'stability_duration': 0,    # seconds, duration pointing must be maintained
                    'max_slew_rate': 3.0,         # degrees/second, maximum allowed slew rate
                    'nominal_slew_rate': 2.0      # degrees/second, target slew rate during maneuvers
                }
            },
            'power': {
                'solar_panels': {
                    'pX': {'area': 0.03},  # 10x30cm = 0.03 m²
                    'nX': {'area': 0.03},
                    'pY': {'area': 0.03},
                    'nY': {'area': 0.03}
                },
                'solar_efficiency': 0.3,  # 30% efficient solar cells
                'solar_flux': 1361.0     # W/m² (solar constant)
            },
            'eo_camera': {  # EO Camera Configuration
                'resolution': 4096,  # pixels
                'swath': 25.0,       # km, width of the swath in kilometers
            }
        }
    }   
}

# Universe Configuration
UNIVERSE_CONFIG = {
    'perturbations': {
        'gravity': True,        # N-body gravitational forces
        'drag': True,          # Atmospheric drag
        'srp': True,           # Solar radiation pressure
        'magnetic': True       # Magnetic torques
    },
    'space_weather': {
        'F107': 150.0,              # Solar F10.7 radio flux (solar radio flux at 10.7 cm wavelength)
        'F107A': 150.0,             # 81-day average of F10.7
        'magnetic_index': 4.0,      # Geomagnetic AP index (0-400, measure of magnetic field disturbance)
        'solar_cycle_phase': 0.5    # 0-1, current phase of solar cycle
    },
    'max_altitude_for_drag': 1000.0  # km, altitude above which to ignore atmospheric drag
}

# Orbit Configuration
ORBIT_CONFIG = {
    'spacecraft': {  # Matches spacecraft name in SPACECRAFT_CONFIG
        'epoch': SIM_CONFIG['mission_start_time'],
        'elements': {
            'semi_major_axis': 6878.0,    # km (500km altitude + Earth radius 6378km)
            'eccentricity': 0.0001,       # Nearly circular (0.0001 is close enough)    
            'inclination': 97.4,          # degrees (SSO inclination for 500km)
            'raan': 22.5,                 # degrees (typically chosen for LTAN)
            'arg_perigee': 0.0,           # degrees (0° is prograde, 90° is prograde circular)
            'true_anomaly': 0.0           # degrees (0° is perigee, 180° is apogee) 
        }
    }
}
