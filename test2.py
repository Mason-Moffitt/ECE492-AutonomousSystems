from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import argparse
import time
import math

rcin_5 = 0
rcin_6 = 0

parser = argparse.ArgumentParser(description='Takeoff and land at a given GPS coordinate.')
parser.add_argument('--connect', default='127.0.0.1:14550', help="Connection string")
parser.add_argument('--landing_lat', type=float, required=True, help="Landing latitude")
parser.add_argument('--landing_lon', type=float, required=True, help="Landing longitude")
args = parser.parse_args()

print(f"Connecting to vehicle on {args.connect}...")

# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=False)

print('Succesfully connected to vehicle')

@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_5
    global rcin_6
    rcin_5 = message.chan5_raw
    rcin_6 = message.chan6_raw

def calculate_bearing(loc1, loc2):
    """
    Calculate the bearing from loc1 to loc2 assuming a flat Earth.
    """
    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(loc1.lat)
    lat2 = math.radians(loc2.lat)
    lon1 = math.radians(loc1.lon)
    lon2 = math.radians(loc2.lon)

    # Differences in coordinates
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1

    # Calculate bearing
    bearing = math.atan2(d_lon, d_lat)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360  # Normalize to 0-360 degrees

    return bearing

def get_location_meter(original_location, dNorth, dEast):
    """
    Calculate a new location given displacements northward and eastward,
    assuming a flat Earth.
    """
    # Approximate meters per degree at the equator
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * math.cos(math.radians(original_location.lat))

    # Calculate new latitude and longitude
    new_lat = original_location.lat + (dNorth / meters_per_deg_lat)
    new_lon = original_location.lon + (dEast / meters_per_deg_lon)

    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def send_guided_command(vehicle, lat, lon, alt):
    """Send a simple guided command (example: climb 50m forward)."""
    #set_loiter_radius(vehicle, 5)
    # Fly to the user-input location
    #fly_to_coordinate(vehicle, lat, lon, alt)
    #time.sleep(1)
    
    print("Setting throttle to 0%")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 3, -1, 1, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_loiter_radius(vehicle, radius):
    print(f"Setting loiter radius to {radius} meters...")
    vehicle.parameters['WP_LOITER_RAD'] = radius
    while abs(vehicle.parameters['WP_LOITER_RAD'] - radius) > 0.1:
        print(" Waiting for parameter update...")
        time.sleep(1)
    print(f"Loiter radius set to {vehicle.parameters['WP_LOITER_RAD']} meters.")


def wait_for_guided_trigger():
    print("Waiting for switch to AUTO... (Channel 6 high, Throttle low)")
    while True:
        throttle = rcin_5
        mode_switch = rcin_6

        if mode_switch > 1500 and throttle < 1200:
            print("Trigger condition met. Switching to AUTO mode.")
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name == "AUTO":
                print("Waiting for AUTO mode...")
                time.sleep(1)
            return
        time.sleep(0.5)

def get_distance_metres(location1, location2):
    dlat = location2.lat - location1.lat
    dlon = location2.lon - location1.lon
    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5

def fly_to_coordinate(vehicle, lat, lon, alt):
    print(f"Flying to target: lat={lat}, lon={lon}, alt={alt}")
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location)

def monitor_manual_override(vehicle, target_lat, target_lon, target_alt):
    print("Monitoring RC for manual override (Channel 6 low)...")
    while True:
        #Log information
        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
        distance = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        alt_diff = abs(vehicle.location.global_relative_frame.alt - target_alt)
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"DEBUG: Distance={distance:.1f}m, Alt diff={alt_diff:.1f}m, Current[{current_lat:.7f},{current_lon:.7f},{current_alt:.1f}m], Target=[{target_lat:.7f},{target_lon:.7f},{target_alt:.1f}m]")

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
   
    current_location = vehicle.location.global_relative_frame

    # Define landing location
    landing_location = LocationGlobalRelative(args.landing_lat, args.landing_lon, 0)

    # Calculate bearing from landing to current location
    bearing = calculate_bearing(landing_location, current_location)

    # Calculate pre-approach waypoint: 500m from landing at 50m altitude
    distance_pre = 500
    north_offset_pre = distance_pre * math.cos(math.radians(bearing))
    east_offset_pre = distance_pre * math.sin(math.radians(bearing))
    pre_approach_location = get_location_meter(landing_location, north_offset_pre, east_offset_pre)
    pre_approach_location.alt = 50

    # Calculate approach waypoint: 200m from landing at 20m altitude
    distance_approach = 200
    north_offset_approach = distance_approach * math.cos(math.radians(bearing))
    east_offset_approach = distance_approach * math.sin(math.radians(bearing))
    approach_location = get_location_meter(landing_location, north_offset_approach, east_offset_approach)
    approach_location.alt = 20

    # Create mission
    cmds = vehicle.commands
    cmds.clear()

    # Add pre-approach waypoint
    cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pre_approach_location.lat, pre_approach_location.lon, pre_approach_location.alt)
    cmds.add(cmd1)

    # Add approach waypoint
    cmd2 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, approach_location.lat, approach_location.lon, approach_location.alt)
    cmds.add(cmd2)

    # Add NAV_LAND
    cmd3 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, landing_location.lat, landing_location.lon, 0)
    cmds.add(cmd3)

    cmds.upload()
    print("Mission uploaded") 
    
    # Let pilot fly manually until switch condition is met
    wait_for_guided_trigger()
    
    # Execute autonomous task
    send_guided_command(vehicle, 35.771355, -78.674330, 50)

    # Monitor for manual override anytime during flight
    monitor_manual_override(vehicle, 35.771355, -78.674330, 50)

    print("Autonomous command completed or overridden.")

    vehicle.close()

if __name__ == "__main__":
    main()

