from datetime import datetime, timezone

LOGGER_CONFIG = {
    'level': 'INFO',  # Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
}

# Simulator Configuration
SIM_CONFIG = {
    'epoch': datetime(2000, 1, 1, 12, 0, 0, tzinfo=timezone.utc),  # Epoch time (don't change this!!!)
    'mission_start_time': datetime(2025, 1, 1, 12, 30, 0, tzinfo=timezone.utc),  # Mission start time
    'time_step': 1.0,                                     # Simulation time step in seconds
    'download_directory': './BOSSY/MCS/download/',
    'upload_directory': './BOSSY/MCS/upload/',
    'google_maps_api_key': 'AIzaSyDL__brVoZ4VY72_ZnRl5MhLnWLpuP4bsA'
}

# Universe Configuration
UNIVERSE_CONFIG = {
    'perturbations': {
        'J2': True,           # J2 perturbation
        'atmospheric': True,   # Atmospheric drag
        'radiation': False    # Solar radiation pressure (not implemented)
    },
    'atmosphere': {
        'density_model': 'exponential',  # Which atmospheric model to use
        'F107': 150.0,                   # Solar F10.7 flux
        'Ap': 4.0                        # Geomagnetic index
    }
}

# Orbit Configuration
ORBIT_CONFIG = {
    'spacecraft': {  # Matches spacecraft name in SPACECRAFT_CONFIG
        'epoch': SIM_CONFIG['mission_start_time'],
        'elements': {
            'semi_major_axis': 6865.75,    # km (500km altitude + Earth radius 6378km)
            'eccentricity': 0.00074436,       # Nearly circular (0.0001 is close enough)    
            'inclination': 97.4050,          # degrees (SSO inclination for 500km)
            'raan': 252.10,                 # degrees (typically chosen for LTAN)
            'arg_perigee': 294.0372,           # degrees (0° is prograde, 90° is prograde circular)
            'true_anomaly': 162.46           # degrees (0° is perigee, 180° is apogee) 
        }
    }
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
                'angular_rate': [5.1, 4.6, 3.8],     # float_deg_s[3]: Angular rates [x,y,z] in deg/s
                'position': [0.0, 0.0, 500.0],       # [float_deg,float_deg,float_km]: [lat,lon,alt] (doesn't need to match orbit params)
                'eclipse': 0         # Eclipse_Type (uint8): 0=DAY, 1=NIGHT
            },

            # COMMS Initial State
            'comms': {
                'state': 2,          # SubsystemState_Type (uint8): 0=OFF, 1=IDLE, 2=ACTIVE, 3=ERROR
                'temperature': 20,    # int8_degC: Operating temperature in degrees Celsius
                'heater_setpoint': 25,# int8_degC: Temperature setpoint for heater control
                'rx_power_draw': 1.0, # float_W: Power consumption in Watts
                'txrx_power_draw': 3.7,    # float_W: Power consumption in Watts
                'mode': 0,           # CommsMode_Type (uint8): 0=RX, 1=TXRX
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
                'pointing_requirements': {
                    'accuracy_threshold': 2.0,     # degrees, threshold between SLEWING and POINTING
                    'stability_duration': 0,    # seconds, duration pointing must be maintained
                    'max_slew_rate': 3.0,         # degrees/second, maximum allowed slew rate
                    'nominal_slew_rate': 2.0      # degrees/second, target slew rate during maneuvers
                }
            },
            'power': {
                'solar_panels': {
                    'pX': {'area': 0.03},  # m²
                    'nX': {'area': 0.03},  # m²
                    'pY': {'area': 0.03},  # m²
                    'nY': {'area': 0.03}   # m²
                },
                'solar_efficiency': 0.3,    # dimensionless
                'solar_flux': 1361.0       # W/m²
            },
            'eo_camera': {  # EO Camera Configuration
                'resolution': 4096,  # pixels
                'meters_per_pixel': 500,  # Ground resolution in meters/pixel
            }
        }
    }   
}
