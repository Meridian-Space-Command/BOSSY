import struct
from logger import SimLogger
from config import SPACECRAFT_CONFIG, SIM_CONFIG
import numpy as np
import time
import requests
import os
from PIL import Image
from io import BytesIO

class PayloadModule:
    def __init__(self, adcs):
        self.logger = SimLogger.get_logger("PayloadModule")
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['payload']
        
        # Store ADCS reference
        self.adcs_module = adcs
        
        # Initialize PAYLOAD state from config
        self.state = config['state']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.idle_power_draw = config['idle_power_draw']
        self.capturing_power_draw = config['capturing_power_draw']
        self.power_draw = self.idle_power_draw
        self.status = config['status']
        self.storage_path = SPACECRAFT_CONFIG['spacecraft']['initial_state']['datastore']['storage_path']
        
        # Get position from ADCS
        self.latitude = self.adcs_module.latitude
        self.longitude = self.adcs_module.longitude
        self.altitude = self.adcs_module.altitude
        
        # Time configuration
        self.mission_epoch = SIM_CONFIG['epoch']
        self.current_time = SIM_CONFIG['mission_start_time']

        # EO Camera Configuration
        self.eo_camera = SPACECRAFT_CONFIG['spacecraft']['hardware']['eo_camera']
        self.zoom = self.eo_camera['zoom']
        self.resolution = self.eo_camera['resolution']
        
    def get_telemetry(self):
        """Package current PAYLOAD state into telemetry format"""
        if self.status == 0:
            self.power_draw = self.idle_power_draw + np.random.uniform(-0.05, 0.05)
        else:
            self.power_draw = self.capturing_power_draw + np.random.uniform(-0.05, 0.05)
            
        values = [
            np.uint8(self.state),              # SubsystemState_Type (8 bits)
            np.int8(self.temperature),         # int8_degC (8 bits)
            np.int8(self.heater_setpoint),     # int8_degC (8 bits)
            np.float32(self.power_draw),       # float_W (32 bits)
            np.uint8(self.status)              # PayloadStatus_Type (8 bits)
        ]
        
        return struct.pack(">BbbfB", *values)
    
    def update(self, current_time, adcs):
        """Update PAYLOAD state"""
        # Update time
        self.current_time = current_time
        
        # Update position from orbit state (already in correct units)
        self.latitude = adcs.latitude   # degrees
        self.longitude = adcs.longitude # degrees
        self.altitude = adcs.altitude   # km

    def process_command(self, command_id, command_data):
        """Process PAYLOAD commands (Command_ID range 50-59)"""
        self.logger.info(f"Processing PAYLOAD command {command_id}: {command_data.hex()}")
        
        try:
            if command_id == 50:    # PAYLOAD_SET_STATE
                state = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting PAYLOAD state to: {state}")
                self.state = state
                
            elif command_id == 51:   # PAYLOAD_SET_HEATER
                heater = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting PAYLOAD heater to: {heater}")
                self.heater_state = heater
                
            elif command_id == 52:   # PAYLOAD_SET_HEATER_SETPOINT
                setpoint = struct.unpack(">f", command_data)[0]
                self.logger.info(f"Setting PAYLOAD heater setpoint to: {setpoint}°C")
                self.heater_setpoint = setpoint
                
            elif command_id == 53:   # PAYLOAD_IMAGE_CAPTURE
                current_time = self.current_time
                lat = self.latitude
                lon = self.longitude
                alt = self.altitude
                res = self.resolution
                zoom = self.zoom
                self.logger.info(f"Starting image capture with parameters, Lat: {lat}, Lon: {lon}, Alt: {alt}, Res: {res}, zoom: {zoom}")
                if self.state == 2:  # Only if powered ON
                    self._capture_image(current_time, lat, lon, alt, res, zoom)
                
            else:
                self.logger.warning(f"Unknown PAYLOAD command ID: {command_id}")
                
        except struct.error as e:
            self.logger.error(f"Error unpacking PAYLOAD command {command_id}: {e}")

    def process_ats_command(self, command_id, command_data):
        """Process PAYLOAD commands (Command_ID range 50-59)"""
        self.logger.info(f"Processing PAYLOAD command {command_id}: {command_data}")
        
        if command_id == 50:    # PAYLOAD_SET_STATE
            state = int(command_data)
            self.logger.info(f"Setting PAYLOAD state to: {state}")
            self.state = state
            
        elif command_id == 51:   # PAYLOAD_SET_HEATER
            heater = int(command_data)
            self.logger.info(f"Setting PAYLOAD heater to: {heater}")
            self.heater_state = heater
            
        elif command_id == 52:   # PAYLOAD_SET_HEATER_SETPOINT
            setpoint = float(command_data)
            self.logger.info(f"Setting PAYLOAD heater setpoint to: {setpoint}°C")
            self.heater_setpoint = setpoint
            
        elif command_id == 53:   # PAYLOAD_IMAGE_CAPTURE
            current_time = self.current_time
            lat = self.latitude
            lon = self.longitude
            alt = self.altitude
            res = self.resolution
            zoom = self.zoom
            self.logger.info(f"Starting image capture with parameters, Lat: {lat}, Lon: {lon}, Alt: {alt}, Res: {res}, zoom: {zoom}")
            if self.state == 2:  # Only if powered ON
                self._capture_image(current_time, lat, lon, alt, res, zoom)
            
        else:
            self.logger.warning(f"Unknown PAYLOAD command ID: {command_id}")
                
    def _capture_image(self, current_time, lat, lon, alt, res, zoom):
        """Capture an image at the specified MET seconds"""
        mission_start_time = SIM_CONFIG['mission_start_time']
        met_sec = int((current_time - mission_start_time).total_seconds())
        self.logger.debug(f"Current time: {current_time}")
        self.logger.debug(f"Current MET seconds: {met_sec}")
        self.logger.debug(f"Capturing image with parameters: Lat: {lat}, Lon: {lon}, Alt: {alt}, Res: {res}, zoom: {zoom}")
        
        self.logger.debug("Capturing image")
        try:
            self.status = 1  # Set to CAPTURING

            zoom = self.zoom  # google api specific value that is roughly the config.py zoom at equator
            
            # Build API request
            params = {
                'center': f'{lat},{lon}',  # Use spacecraft's actual position
                'zoom': zoom,  # Google Maps API specific parameter
                'size': f'{res}x{res}',  # resolution from config.py (pixels), same for width and height
                'maptype': 'satellite',  # Google Maps API specific parameter
                'key': SIM_CONFIG['google_maps_api_key'],  # Google Maps API key
                'scale': 2  # Request high-resolution tiles
            }

            self.logger.info(f"Capturing {res}x{res} image at position (lat={lat}, lon={lon}) at zoom level {zoom}")
            
            # Create filename that increments with each capture
            filename = f"EO_image_met_{met_sec}.png"
            filepath = os.path.join(self.storage_path, filename)

            # Ensure storage directory exists
            os.makedirs(self.storage_path, exist_ok=True)

            # Make API request
            base_url = "https://maps.googleapis.com/maps/api/staticmap"
            response = requests.get(base_url, params=params)
            img = Image.open(BytesIO(response.content))
            
            # Resize to exactly resxres if needed
            if img.size != (res, res):
                img = img.resize((res, res), Image.Resampling.LANCZOS)
            
            # Save the image in PNG format
            img.save(filepath, 'PNG')
            self.logger.info(f"Saved {img.size} image to {filepath}")
            
            # Update payload status to indicate capture complete
            self.status = 0  # Back to IDLE
            
        except Exception as e:
            self.logger.error(f"Error capturing/saving image: {e}")
            self.status = 0  # Set back to IDLE on error
        return

    def get_power_draw(self):
        """Get current power draw in Watts"""
        return self.power_draw
