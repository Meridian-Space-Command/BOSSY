from logger import setup_logging
setup_logging()  # Must be called before other imports

import time
import signal
import sys
from datetime import timedelta
from apps.spacecraft.cdh import CDHModule
from apps.spacecraft.adcs import ADCSModule
from apps.spacecraft.power import PowerModule
from apps.spacecraft.payload import PayloadModule
from apps.spacecraft.datastore import DatastoreModule
from apps.spacecraft.obc import OBCModule
from apps.spacecraft.comms import CommsModule
from apps.universe.orbit import OrbitPropagator, SimulationEndedException
from apps.universe.environment import Environment
from config import SIM_CONFIG, SPACECRAFT_CONFIG
from logger import SimLogger
from astropy import units as u

class Simulator:
    _sim_time = SIM_CONFIG['mission_start_time']
    _epoch = SIM_CONFIG['epoch']
    
    @classmethod
    def get_sim_time(cls):
        return cls._sim_time
    
    @classmethod
    def get_epoch(cls):
        return cls._epoch

    def __init__(self):
        self.logger = SimLogger.get_logger("Simulator")
        
        # Initialize orbit propagator and environment first
        self.orbit_propagator = OrbitPropagator()
        self.environment = Environment()
        
        # Get initial orbit state to initialize subsystems
        initial_state = self.orbit_propagator.propagate(SIM_CONFIG['mission_start_time'])
        
        # Log initial position
        self.logger.info(f"Initial position: lat={initial_state['lat']:.2f}°, lon={initial_state['lon']:.2f}°, alt={initial_state['alt']:.1f}km")
        
        # Initialize all subsystems with proper initial state
        self.adcs = ADCSModule(initial_state, self.orbit_propagator)  # Pass the existing propagator
        self.comms = CommsModule()
        self.payload = PayloadModule(self.adcs)
        self.datastore = DatastoreModule()
        
        # Initialize power module with all dependencies
        self.power = PowerModule(
            logger=self.logger,
            orbit_propagator=self.orbit_propagator,
            environment=self.environment,
            comms=self.comms,
            payload=self.payload,
            datastore=self.datastore
        )

        self.obc = OBCModule()
        self.cdh = CDHModule()
        
        # Create subsystems dictionary for CDH and telemetry
        self.subsystems = {
            'obc': self.obc,
            'cdh': self.cdh,
            'power': self.power,
            'adcs': self.adcs,
            'comms': self.comms,
            'payload': self.payload,
            'datastore': self.datastore
        }

        self.obc.set_subsystems(self.subsystems)
        
        # Set up COMMS with CDH reference and subsystems for command routing
        self.comms.set_cdh(self.cdh)
        self.comms.set_subsystems(self.subsystems)
        self.comms.start()
        
        # Get simulation configuration
        self.mission_start_time = SIM_CONFIG['mission_start_time']
        self.time_step = SIM_CONFIG['time_step']
        
        # Initialize simulation time
        self.current_time = self.mission_start_time
        self.running = False

        # Initialize OBC uptime
        self.obc.set_uptime(SPACECRAFT_CONFIG['spacecraft']['initial_state']['obc']['uptime'])
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.logger.info(f"Simulator started - Mission start time: {self.mission_start_time}")
        self.logger.info(f"Time step: {self.time_step}s")
        self.logger.info("Press Ctrl+C to stop the simulator")

    def calculate_total_power_draw(self):
        """Calculate total power draw from all subsystems"""
        power_draws = {
            name: subsystem.get_power_draw()
            for name, subsystem in self.subsystems.items()
        }
        
        self.logger.debug("Power draws by subsystem:")
        for name, draw in power_draws.items():
            self.logger.debug(f"  {name}: {draw}W")
            
        total_power = sum(power_draws.values())
        self.logger.debug(f"Total power draw: {total_power}W")
        
        return total_power

    def run(self):
        self.running = True
        
        try:
            while self.running:
                # Update simulation time
                Simulator._sim_time += timedelta(seconds=self.time_step)
                self.current_time = Simulator._sim_time
                
                try:
                    # First propagate orbit to get new state
                    orbit_state = self.orbit_propagator.propagate(self.current_time)
                    
                    # Update subsystems in sequence with orbit state
                    self.adcs.update(self.current_time)
                    self.obc.update(self.current_time)
                    
                    # Calculate total power draw
                    total_power = self.calculate_total_power_draw()
                    self.power.set_total_power_draw(total_power * u.W)  # Add unit
                    self.power.update(self.current_time, self.adcs)
                    
                    # Update payload with current state
                    self.payload.update(self.current_time, self.adcs)
                    
                    # Create and send telemetry
                    tm_packet = self.cdh.create_tm_packet(self.current_time, self.subsystems)
                    self.comms.send_tm_packet(tm_packet)
                    
                    # Update OBC uptime with proper time unit
                    self.obc.set_uptime(self.obc.get_uptime() + self.time_step * u.s)
                    
                except SimulationEndedException as e:
                    self.logger.info(f"Simulation ended: {str(e)}")
                    self.stop()
                    break
                
                # Sleep for time_step
                time.sleep(self.time_step)
                
        except Exception as e:
            self.logger.error(f"Error in simulation loop: {str(e)}")
            self.stop()

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.logger.info("\nShutdown signal received. Stopping simulator...")
        self.stop()
        sys.exit(0)

    def stop(self):
        """Clean shutdown of simulator"""
        self.running = False
        self.logger.info("Stopping COMMS module...")
        self.comms.stop()
        self.logger.info("Simulator stopped")

if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()
