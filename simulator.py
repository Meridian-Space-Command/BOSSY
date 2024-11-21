#!/usr/bin/env python3

import sys
import argparse
from time import sleep

# Import our new simulator package
from simulator import SpacecraftSimulator, UniverseSimulator
from simulator.spacecraft_config import CCSDS_CONFIG

parser = argparse.ArgumentParser(description='Yamcs Spacecraft Simulator')

# telemetry
parser.add_argument('--tm_host', type=str, default='127.0.0.1', help='TM host')
parser.add_argument('--tm_port', type=int, default=10015, help='TM port')
parser.add_argument('-r', '--rate', type=int, default=1, help='Simulation speed multiplier')

# telecommand
parser.add_argument('--tc_host', type=str, default='127.0.0.1', help='TC host')
parser.add_argument('--tc_port', type=int, default=10025, help='TC port')

args = vars(parser.parse_args())

def main():
    # Update CCSDS configuration with command line arguments
    CCSDS_CONFIG.update({
        'tm_host': args['tm_host'],
        'tm_port': args['tm_port'],
        'tc_host': args['tc_host'],
        'tc_port': args['tc_port'],
    })

    # Initialize simulators
    universe = UniverseSimulator()
    spacecraft = SpacecraftSimulator(universe)
    
    # Set simulation speed
    universe.set_speed_multiplier(args['rate'])
    
    # Print startup message
    sys.stdout.write(f"Using simulation rate of {args['rate']}x realtime, ")
    sys.stdout.write(f"TM host={args['tm_host']}, TM port={args['tm_port']}, ")
    sys.stdout.write(f"TC host={args['tc_host']}, TC port={args['tc_port']}\r\n")
    
    try:
        prev_status = None
        while True:
            # Get current status
            status = spacecraft.print_status()
            
            # Only print if status has changed
            if status != prev_status:
                sys.stdout.write('\r')
                sys.stdout.write(status)
                sys.stdout.flush()
                prev_status = status
                
            sleep(0.5)
            
    except KeyboardInterrupt:
        sys.stdout.write('\n')
        sys.stdout.flush()
        spacecraft.stop()

if __name__ == '__main__':
    main()