import struct
from logger import SimLogger
from config import SPACECRAFT_CONFIG, SIM_CONFIG
import numpy as np
import os
import shutil
import time

class DatastoreModule:
    def __init__(self):
        self.logger = SimLogger.get_logger("DatastoreModule")
        config = SPACECRAFT_CONFIG['spacecraft']['initial_state']['datastore']
        
        # Initialize DATASTORE state from config
        self.state = config['state']
        self.temperature = config['temperature']
        self.heater_setpoint = config['heater_setpoint']
        self.power_draw = config['power_draw']
        
        # Storage parameters
        self.storage_path = config['storage_path']
        self.storage_total = config['storage_total']
        self.storage_used = 0
        self.files_stored = 0
        self.storage_remaining = self.storage_total
        self.mode = config['mode']
        self.transfer_progress = 0

        # Ensure storage directory exists
        os.makedirs(self.storage_path, exist_ok=True)
        
        # Initialize storage stats
        self._update_storage_stats()

    def _update_storage_stats(self):
        """Update storage statistics"""
        try:
            self.files = os.listdir(self.storage_path)
            self.files_stored = len(self.files)
            self.storage_used = sum(
                os.path.getsize(os.path.join(self.storage_path, file)) 
                for file in self.files
            )
            self.storage_remaining = self.storage_total - self.storage_used
        except Exception as e:
            self.logger.error(f"Error updating storage stats: {e}")

    def get_telemetry(self):
        """Package current DATASTORE state into telemetry format"""
        self._update_storage_stats()
        self.power_draw = self.power_draw + np.random.uniform(-0.05, 0.05)

        values = [
            np.uint8(self.state),                # SubsystemState_Type (8 bits)
            np.int8(self.temperature),           # int8_degC (8 bits)
            np.int8(self.heater_setpoint),       # int8_degC (8 bits)
            np.float32(self.power_draw),         # float_W (32 bits)
            np.float32(self.storage_remaining),  # float_MB (32 bits)
            np.uint32(self.files_stored),        # uint32 (32 bits)
            np.uint8(self.mode),                 # DatastoreMode_Type (8 bits)
            np.float32(self.transfer_progress)   # float (32 bits)
        ]
        
        return struct.pack(">BbbffIBf", *values)
        
    def process_command(self, command_id, command_data):
        """Process DATASTORE commands (Command_ID range 60-69)"""
        self.logger.info(f"Processing DATASTORE command {command_id}: {command_data.hex()}")
        
        try:
            if command_id == 60:    # DATASTORE_SET_STATE
                state = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting DATASTORE state to: {state}")
                self.state = state
                
            elif command_id == 61:   # DATASTORE_SET_HEATER
                heater = struct.unpack(">B", command_data)[0]
                self.logger.info(f"Setting DATASTORE heater to: {heater}")
                self.heater_state = heater
                
            elif command_id == 62:   # DATASTORE_SET_HEATER_SETPOINT
                setpoint = struct.unpack(">f", command_data)[0]
                self.logger.info(f"Setting DATASTORE heater setpoint to: {setpoint}°C")
                self.heater_setpoint = setpoint
                
            elif command_id == 63:   # DATASTORE_CLEAR_DATASTORE
                self.logger.info("Clearing all files from DATASTORE")
                for file in os.listdir(self.storage_path):
                    try:
                        os.remove(os.path.join(self.storage_path, file))
                    except Exception as e:
                        self.logger.error(f"Error removing file {file}: {e}")
                self._update_storage_stats()
                
            elif command_id == 64:   # DATASTORE_DOWNLOAD_FILE
                filename = command_data.decode('utf-8').strip('\x00')
                self._download_file(filename)
            
            elif command_id == 65:   # DATASTORE_UPLOAD_FILE
                filename = command_data.decode('utf-8').strip('\x00')
                self._upload_file(filename)
                
            else:
                self.logger.warning(f"Unknown DATASTORE command ID: {command_id}")
                
        except struct.error as e:
            self.logger.error(f"Error unpacking DATASTORE command {command_id}: {e}")

    def process_ats_command(self, command_id, command_data):
        """Process DATASTORE commands (Command_ID range 60-69)"""
        self.logger.info(f"Processing DATASTORE command {command_id}: {command_data}")
        
        if command_id == 60:    # DATASTORE_SET_STATE
            state = int(command_data)
            self.logger.info(f"Setting DATASTORE state to: {state}")
            self.state = state
            
        elif command_id == 61:   # DATASTORE_SET_HEATER
            heater = int(command_data)
            self.logger.info(f"Setting DATASTORE heater to: {heater}")
            self.heater_state = heater
            
        elif command_id == 62:   # DATASTORE_SET_HEATER_SETPOINT
            setpoint = float(command_data)  
            self.logger.info(f"Setting DATASTORE heater setpoint to: {setpoint}°C")
            self.heater_setpoint = setpoint
            
        elif command_id == 63:   # DATASTORE_CLEAR_DATASTORE
            self.logger.info("Clearing all files from DATASTORE")
            for file in os.listdir(self.storage_path):
                try:
                    os.remove(os.path.join(self.storage_path, file))
                except Exception as e:
                    self.logger.error(f"Error removing file {file}: {e}")
            self._update_storage_stats()
            
        elif command_id == 64:   # DATASTORE_DOWNLOAD_FILE
            filename = command_data
            self._download_file(filename)

        elif command_id == 65:   # DATASTORE_UPLOAD_FILE
            filename = command_data
            self._upload_file(filename)
            
        else:
            self.logger.warning(f"Unknown DATASTORE command ID: {command_id}")

    def _download_file(self, filename):
        """Handle file download process"""
        self.logger.info(f"Initiating download of file: {filename}")
        filepath = os.path.join(self.storage_path, filename)
        
        if not os.path.exists(filepath):
            self.logger.warning(f"File not found: {filename}")
            return
        
        if self.mode != 0:
            self.logger.warning("DATASTORE is busy, cannot download file")
            return
            
        try:
            self.mode = 1  # DOWNLOADING
            file_size = os.path.getsize(filepath)
            bitrate = SPACECRAFT_CONFIG['spacecraft']['initial_state']['comms']['downlink_bitrate']
            total_duration = (file_size * 8) / bitrate
            
            # Simulate transfer
            for progress in range(0, 101, 1):
                self.transfer_progress = progress
                self.logger.info(f"Transfer progress: {progress}%")
                time.sleep(total_duration / 100)
                
            # Copy file to download directory
            download_path = os.path.join('../../', SIM_CONFIG['download_directory'])
            shutil.copy(filepath, os.path.join(download_path, filename))
            
        except Exception as e:
            self.logger.error(f"Error transferring file: {e}")
        finally:
            self.mode = 0  # IDLE
            self.transfer_progress = 0

    def _upload_file(self, filename):
        """Handle file upload process"""
        self.logger.info(f"Initiating upload of file: {filename}")
        filepath = os.path.join('../../', SIM_CONFIG['upload_directory'], filename)
        
        if not os.path.exists(filepath):
            self.logger.warning(f"File not found: {filename}")
            return
        
        if self.mode != 0:
            self.logger.warning("DATASTORE is busy, cannot upload file")
            return
            
        try:
            self.mode = 2  # UPLOADING
            file_size = os.path.getsize(filepath)
            bitrate = SPACECRAFT_CONFIG['spacecraft']['initial_state']['comms']['uplink_bitrate']
            total_duration = (file_size * 8) / bitrate
            
            # Simulate transfer
            for progress in range(0, 101, 1):
                self.transfer_progress = progress
                self.logger.info(f"Transfer progress: {progress}%")
                time.sleep(total_duration / 100)
                
            # Copy file to storage directory
            shutil.copy(filepath, os.path.join(self.storage_path, filename))
            
        except Exception as e:
            self.logger.error(f"Error transferring file: {e}")
        finally:
            self.mode = 0  # IDLE
            self.transfer_progress = 0

    def get_power_draw(self):
        """Get current power draw in Watts"""
        return self.power_draw
