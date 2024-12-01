from datetime import datetime
from config import SIM_CONFIG

class SimTime:
    _current_time = SIM_CONFIG['mission_start_time']
    
    @classmethod
    def get_time(cls):
        return cls._current_time
        
    @classmethod
    def set_time(cls, time):
        cls._current_time = time 