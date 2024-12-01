import logging
import os
from datetime import datetime
import warnings
import urllib3

def setup_logging():
    """Configure warnings and logging before any other imports"""
    # Configure warnings
    warnings.filterwarnings('ignore', category=urllib3.exceptions.NotOpenSSLWarning)
    warnings.filterwarnings('ignore', category=DeprecationWarning)
    warnings.filterwarnings('ignore', category=FutureWarning)
    
    # Suppress specific module logs
    logging.getLogger('numba').setLevel(logging.WARNING)
    logging.getLogger('urllib3').setLevel(logging.WARNING)
    logging.getLogger('matplotlib').setLevel(logging.WARNING)

class SimLogger:
    _logger = None
    
    @classmethod
    def get_logger(cls, name="SimLogger"):
        if cls._logger is None:
            # Create logger
            cls._logger = logging.getLogger(name)
            cls._logger.setLevel(logging.INFO)
            
            # Create log directory if it doesn't exist
            if not os.path.exists('logs'):
                os.makedirs('logs')
                
            # Create file handler
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            fh = logging.FileHandler(f'logs/sim_{timestamp}.log')
            fh.setLevel(logging.INFO)
            
            # Create console handler
            ch = logging.StreamHandler()
            ch.setLevel(logging.INFO)
            
            # Create formatter
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            fh.setFormatter(formatter)
            ch.setFormatter(formatter)
            
            # Add handlers to logger
            cls._logger.addHandler(fh)
            cls._logger.addHandler(ch)
            
            cls._logger.info(f"Log file created at: {fh.baseFilename}")
            cls._logger.info(f"Logging level set to: {logging.getLevelName(cls._logger.level)}")
            
        return cls._logger