# Import required libraries for drone control and utilities
from dronekit import connect, VehicleMode, LocationGlobalRelative 
from pymavlink import mavutil 
import argparse  
import time  
import math  

# Global variables to store RC channel values
rcin_5 = 0 
rcin_6 = 0

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# Acquire connection_string from command-line arguments
connection_string = args.connect 

# Exit if no connection string specified
if not connection_string:  # Check if connection string is missing
    sys.exit('Please specify connection string')  # Exit with an error message if not provided

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True) 

print("Successfully connected to vehicle.")

@vehicle.on_message('RC_CHANNELS')  # Trigger this function when an RC_CHANNELS message is received
def rc_listener(self, name, message):
    global rcin_5
    global rcin_6
    rcin_5 = message.chan5_raw
    rcin_6 = message.chan6_raw 

# Function to send a guided command to the vehicle
def send_guided_command(vehicle, lat, lon, alt):
    set_loiter_radius(vehicle, 5) 
    # Fly to the user-input location
    fly_to_coordinate(vehicle, lat, lon, alt)
    time.sleep(1)
    
    print("Setting throttle to 0%")  # Indicate throttle is being set to zero
    msg = vehicle.message_factory.command_long_encode( 
        0, 0, 
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 0, -1, 1, 0, 0, 0, 0  # Parameters: set throttle to 1%
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to set the vehicle's loiter radius
def set_loiter_radius(vehicle, radius):  # Takes vehicle object and desired radius as parameters
    print(f"Setting loiter radius to {radius} meters...") 
    vehicle.parameters['WP_LOITER_RAD'] = radius 
    while abs(vehicle.parameters['WP_LOITER_RAD'] - radius) > 0.1: 
        print(" Waiting for parameter update...")  
        time.sleep(1)
    print(f"Loiter radius set to {vehicle.parameters['WP_LOITER_RAD']} meters.")  

# Function to wait for the trigger to switch to GUIDED mode
def wait_for_guided_trigger(): 
    print("Waiting for switch to GUIDED... (Channel 6 high, Throttle low)") 
    while True:  
        throttle = rcin_5  
        mode_switch = rcin_6  

        if mode_switch > 1500 and throttle < 1200: 
            print("Trigger condition met. Switching to GUIDED mode.") 
            vehicle.mode = VehicleMode("GUIDED") 
            while not vehicle.mode.name == "GUIDED": 
                print("Waiting for GUIDED mode...")
                time.sleep(1)  
            return 
        time.sleep(0.5) 

# Function to calculate distance between two locations
def get_distance_metres(location1, location2): 
    dlat = location2.lat - location1.lat  
    dlon = location2.lon - location1.lon  
    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5 

# Function to command the vehicle to fly to a specific coordinate
def fly_to_coordinate(vehicle, lat, lon, alt): 
    print(f"Flying to target: lat={lat}, lon={lon}, alt={alt}") 
    location = LocationGlobalRelative(lat, lon, alt)  
    vehicle.simple_goto(location)  

# Function to monitor RC inputs for manual override
def monitor_manual_override(vehicle, target_lat, target_lon, target_alt):
    print("Monitoring RC for manual override (Channel 6 low)...") 
    while True: 
        # Logging and Debugging information
        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt) 
        distance = get_distance_metres(vehicle.location.global_relative_frame, target_location) 
        alt_diff = abs(vehicle.location.global_relative_frame.alt - target_alt)
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"DEBUG: Distance={distance:.1f}m, Alt diff={alt_diff:.1f}m, Current[{current_lat:.7f},{current_lon:.7f},{current_alt:.1f}m], Target=[{target_lat:.7f},{target_lon:.7f},{target_alt:.1f}m]")  # Log debug info

        # Check for manual override
        throttle = rcin_5 
        mode_switch = rcin_6  
        if mode_switch < 1500 or throttle > 1200:
            print("Manual override detected! Switching to MANUAL.") 
            vehicle.mode = VehicleMode("MANUAL") 
            while not vehicle.mode.name == "MANUAL": 
                print("Waiting for MANUAL mode...")
                time.sleep(1)
            return 
        time.sleep(0.5)

# Main function to orchestrate the script's logic
def main():
    print("Setting to manual mode") 
    vehicle.mode = VehicleMode("MANUAL") 
    while not vehicle.mode.name == "MANUAL":
        print("Waiting for MANUAL mode...")  
        time.sleep(1)  
    time.sleep(0.5) 

    print("Waiting for arming...") 
    while not vehicle.armed: 
        time.sleep(1) 
    print("Vehicle is armed. Manual flight assumed.")
    
    # Let pilot fly manually until switch condition is met
    wait_for_guided_trigger()  # Wait for trigger to switch to GUIDED mode
    
    # Execute autonomous task
    send_guided_command(vehicle, 35.771355, -78.674330, 50)  # Send command to fly to specific coordinates

    # Monitor for manual override anytime during flight
    monitor_manual_override(vehicle, 35.771355, -78.674330, 50)  # Monitor RC inputs for override

    print("Autonomous command completed or overridden.")  # Indicate script completion or override

    vehicle.close()

# Run the main function if this script is executed directly
if __name__ == "__main__": 
    main()