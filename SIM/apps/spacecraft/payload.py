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
        self.meters_per_pixel = self.eo_camera['meters_per_pixel']
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
        """Process PAYLOAD commands"""
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
                self.logger.info("Starting image capture")
                if self.state == 2:  # Only if powered ON
                    if self.adcs_module.mode != 3:  # 3 = NADIR
                        self.logger.warning("Image capture rejected: ADCS must be in NADIR mode")
                        return
                        
                    if self.adcs_module.status != 2:  # 2 = POINTING
                        self.logger.warning("Image capture rejected: ADCS must achieve stable pointing first")
                        return
                        
                    self.capture_image(self.current_time)
                
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
            mpp = self.meters_per_pixel
            self.logger.info(f"Starting image capture with parameters, Lat: {lat}, Lon: {lon}, Alt: {alt}, Res: {res}, Resolution: {mpp} m/px")
            if self.state == 2:  # Only if powered ON
                self._capture_image(current_time, lat, lon, alt, res, mpp)
            
        else:
            self.logger.warning(f"Unknown PAYLOAD command ID: {command_id}")
                
    def _get_zoom_limits(self, lat):
        """Calculate minimum and maximum zoom levels for given latitude
        Based on Google Maps zoom level calculations
        """
        # Google Maps limits
        MAX_EQUATOR_ZOOM = 21
        MIN_EQUATOR_ZOOM = 3  # Base minimum zoom
        
        # Latitude adjustment
        lat_rad = abs(lat) * np.pi / 180.0
        
        # Max zoom decreases with latitude
        max_zoom = MAX_EQUATOR_ZOOM - np.log2(1 + np.sin(lat_rad)) / np.log2(2)
        
        # Min zoom increases with latitude to prevent excessive distortion
        # This is approximate based on observed Google Maps behavior
        min_zoom = MIN_EQUATOR_ZOOM + np.log2(1 + np.sin(lat_rad)) / np.log2(2)
        
        return int(min_zoom), int(max_zoom)

    def _meters_per_pixel_to_zoom(self, meters_per_pixel, lat):
        """Convert desired ground resolution to Google Maps zoom level"""
        EARTH_CIRCUMFERENCE = 156543.03392  # meters per pixel at zoom 0 at equator
        
        lat_rad = abs(lat) * np.pi / 180.0
        zoom = np.log2(EARTH_CIRCUMFERENCE * np.cos(lat_rad) / meters_per_pixel)
        
        # Get zoom limits for this latitude
        min_zoom, max_zoom = self._get_zoom_limits(lat)
        
        # Constrain to valid zoom levels
        zoom = np.clip(round(zoom), min_zoom, max_zoom)
        
        # Calculate actual achieved resolution
        actual_mpp = EARTH_CIRCUMFERENCE * np.cos(lat_rad) / (2**zoom)
        
        if abs(actual_mpp - meters_per_pixel) > 0.1 * meters_per_pixel:  # More than 10% difference
            self.logger.warning(f"Requested resolution {meters_per_pixel:.1f} m/px adjusted to {actual_mpp:.1f} m/px due to zoom constraints")
        
        return int(zoom)

    def _capture_image(self, current_time, lat, lon, alt, res, meters_per_pixel):
        """Capture an image at the specified MET seconds"""
        mission_start_time = SIM_CONFIG['mission_start_time']
        met_sec = int((current_time - mission_start_time).total_seconds())

        try:
            self.status = 1  # Set to CAPTURING
            
            # Convert desired resolution to zoom level
            zoom = self._meters_per_pixel_to_zoom(meters_per_pixel, lat)
            
            # Build API request
            params = {
                'center': f'{lat},{lon}',
                'zoom': zoom,
                'size': f'{res}x{res}',
                'maptype': 'satellite',
                'key': SIM_CONFIG['google_maps_api_key'],
                'scale': 2
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
            
            # Check if request was successful
            if response.status_code != 200:
                try:
                    error_json = response.json()  # Try to parse error response as JSON
                    self.logger.error(f"Google Maps API error ({response.status_code}): {error_json}")
                except:
                    # If can't parse JSON, log the raw text
                    self.logger.error(f"Google Maps API error ({response.status_code}): {response.text}")
                return None
            
            # Check content type to ensure we got an image
            if 'image' not in response.headers.get('content-type', ''):
                self.logger.error(f"Received non-image response: {response.headers.get('content-type')}")
                return None
            
            img = Image.open(BytesIO(response.content))
            
            # Check for small image size which might indicate an error tile
            if img.size[0] < 100 or img.size[1] < 100:  # Arbitrary small size check
                self.logger.error("Received error tile or invalid image size")
                return None
            
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

    def capture_image(self, current_time):
        """Capture an image using current position and attitude"""
        try:
            # Check ADCS mode and status
            if self.adcs_module.mode != 3:  # 3 = NADIR
                self.logger.warning("Cannot capture image: ADCS must be in NADIR mode")
                return None
            
            if self.adcs_module.status != 2:  # 2 = POINTING
                self.logger.warning("Cannot capture image: ADCS must achieve stable pointing first")
                return None
            
            # Get current position from ADCS
            lat = self.adcs_module.latitude
            lon = self.adcs_module.longitude
            alt = self.adcs_module.altitude
            is_eclipse = bool(self.adcs_module.eclipse)
            
            # Capture image using existing method
            self._capture_image(current_time, lat, lon, alt, self.resolution, self.meters_per_pixel)
            
            # Get the image path
            met_sec = int((current_time - SIM_CONFIG['mission_start_time']).total_seconds())
            filename = f"EO_image_met_{met_sec}.png"
            filepath = os.path.join(self.storage_path, filename)
            
            # If in eclipse, modify the image with night overlay
            if is_eclipse:
                # Open image and apply night effect
                image = Image.open(filepath)
                overlay = Image.new('RGBA', image.size, (0, 0, 0, 204))  # 204 is 80% of 255
                image = Image.alpha_composite(image.convert('RGBA'), overlay)
                image = image.convert('RGB')  # Convert back to RGB
                
                # Save modified image back to original path
                image.save(filepath, 'PNG')
                self.logger.info(f"Image captured at lat={lat:.2f}°, lon={lon:.2f}°, alt={alt:.1f}km (night)")
            else:
                self.logger.info(f"Image captured at lat={lat:.2f}°, lon={lon:.2f}°, alt={alt:.1f}km")
                
        except Exception as e:
            self.logger.error(f"Error capturing image: {str(e)}")
            return None
