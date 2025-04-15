from __future__ import print_function

import math
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto with precision auto landing.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# Acquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

print('Successfully connected to vehicle')

# Configure precision landing parameters (LiDAR and landing settings)
def configure_precision_landing():
    """
    Set up LiDAR and landing parameters for precision auto landing.
    Assumes downward-facing LiDAR and airspeed sensor.
    """
    try:
        # THR_MAX to default (100)
        vehicle.parameters['THR_MAX'] = 100
        print('Throttle limit 100%')

        # Enable rangefinder for landing
        vehicle.parameters['RNGFND_LANDING'] = 1
        print('Rangefinder enabled for landing')

        # Set rangefinder orientation to downward-facing
        vehicle.parameters['RNGFND_LND_ORNT'] = 25
        print('Rangefinder orientation set to downward')

        # Set minimum range for LiDAR (e.g., PulsedLight)
        vehicle.parameters['RNGFND1_MIN'] = 1.5
        print('Rangefinder minimum range set to 1.5m')

        # Configure landing approach parameters
        vehicle.parameters['TECS_LAND_ARSPD'] = 15  # Target landing airspeed (m/s)
        vehicle.parameters['LAND_PF_ALT'] = 12     # Pre-flare altitude (m)
        vehicle.parameters['LAND_PF_ARSPD'] = 12   # Pre-flare airspeed (m/s)
        vehicle.parameters['LAND_FLARE_ALT'] = 2   # Flare altitude (m)
        print('Landing parameters configured: airspeed=15m/s, pre-flare=12m, flare=2m')

        # Optional: Enable terrain following for irregular terrain
        # vehicle.parameters['TERRAIN_FOLLOW'] = 1  # Uncomment if needed
        # print('Terrain following enabled')

    except Exception as e:
        print(f'Error setting parameters: {e}')

# Set up precision landing parameters before flight
configure_precision_landing()

# Start mission
if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
    vehicle.mode = VehicleMode("TAKEOFF")

# Wait for pilot before proceeding
print('Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('Armed...')
vehicle.mode = VehicleMode("TAKEOFF")
print('Taking off...')

time.sleep(15)
vehicle.mode = VehicleMode("AUTO")
print('Auto mode')

time.sleep(5)
vehicle.mode = VehicleMode("AUTOLAND")
print('Auto land')

# Set THR_MAX to 0 to prevent any throttle output during autoland
vehicle.parameters['THR_MAX'] = 0
print('Throttle limited to 0%')

# Wait until vehicle disarms or exits AUTOLAND
while vehicle.mode.name == "AUTOLAND" and vehicle.armed:
    time.sleep(1)

# Restore THR_MAX to default (100) after autoland completes
vehicle.parameters['THR_MAX'] = 100
print('Throttle limit restored')

# Close vehicle connection
vehicle.close()
print('Vehicle connection closed')