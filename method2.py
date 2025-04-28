from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import argparse
import math

import math

def calculate_bearing(loc1, loc2):
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
    # Approximate meters per degree at the equator
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * math.cos(math.radians(original_location.lat))

    # Calculate new latitude and longitude
    new_lat = original_location.lat + (dNorth / meters_per_deg_lat)
    new_lon = original_location.lon + (dEast / meters_per_deg_lon)

    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def wait_for_manual_arm(vehicle):
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle is armed!")

def set_takeoff_mode(vehicle):
    print("Setting mode to TAKEOFF...")
    vehicle.mode = VehicleMode("TAKEOFF")
    while vehicle.mode != "TAKEOFF":
        print(" Waiting for mode change...")
        time.sleep(1)
    print("Mode is TAKEOFF. Plane should be launching.")

def main():
    parser = argparse.ArgumentParser(description='Takeoff and land at a given GPS coordinate.')
    parser.add_argument('--connect', default='127.0.0.1:14550', help="Connection string")
    parser.add_argument('--landing_lat', type=float, required=True, help="Landing latitude")
    parser.add_argument('--landing_lon', type=float, required=True, help="Landing longitude")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)

    set_takeoff_mode(vehicle)

    # Wait for plane to be airborne
    print("Waiting for plane to build speed and altitude...")
    time.sleep(20)

    # Get current location
    current_location = vehicle.location.global_relative_frame

    # Define landing location
    landing_location = LocationGlobalRelative(args.landing_lat, args.landing_lon, 0)

    # Calculate bearing from landing to current location
    bearing = calculate_bearing(landing_location, current_location)

    # Calculate pre-approach waypoint: 300m from landing at 50m altitude
    distance_pre = 300
    north_offset_pre = distance_pre * math.cos(math.radians(bearing))
    east_offset_pre = distance_pre * math.sin(math.radians(bearing))
    pre_approach_location = get_location_meter(landing_location, north_offset_pre, east_offset_pre)
    pre_approach_location.alt = 50

    # Calculate approach waypoint: 200m from landing at 20m altitude
    distance_approach = 150
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

    # Set mode to AUTO
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != "AUTO":
        print(" Waiting for AUTO mode...")
        time.sleep(1)
    print("Mode is AUTO. Plane should fly to approach and land.")

    # Set throttle to 0%
    print("Setting throttle to 0%")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 3, -1, 1, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

    # Wait for landing to complete
    time.sleep(60) 

    print("Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()