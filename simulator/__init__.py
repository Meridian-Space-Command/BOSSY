"""
Spacecraft Simulator Package

A comprehensive spacecraft simulator including:
- Spacecraft configuration and simulation
- Universe physics and environment simulation
- CCSDS-compliant communications
- YAMCS compatibility

Files:
- spacecraft_config.py: Spacecraft hardware configuration
- spacecraft_sim.py: Spacecraft simulation and subsystems
- universe_config.py: Universe physics configuration
- universe_sim.py: Universe and environment simulation
"""

from .spacecraft_config import *
from .spacecraft_sim import SpacecraftSimulator
from .universe_config import *
from .universe_sim import UniverseSimulator

__version__ = '0.1.0'
__author__ = 'Meridian Space Command'

# Export main simulator classes
__all__ = ['SpacecraftSimulator', 'UniverseSimulator']