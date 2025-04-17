from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import argparse
import math

def calculate_bearing(loc1, loc2):
    lat1 = math.radians(loc1.lat)
    lat2 = math.radians(loc2.lat)
    diff_lon = math.radians(loc2.lon - loc1.lon)
    x = math.sin(diff_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(diff_lon)
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

def get_location_meter(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # meters
    lat = original_location.lat
    lon = original_location.lon
    new_lat = lat + (dNorth / earth_radius) * (180 / math.pi)
    new_lon = lon + (dEast / (earth_radius * math.cos(math.pi * lat / 180))) * (180 / math.pi)
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

    # Configure LiDAR and landing parameters
    print("Configuring LiDAR and landing parameters...")
    vehicle.parameters['RNGFND_LANDING'] = 1
    vehicle.parameters['RNGFND_MIN_CM'] = 50
    vehicle.parameters['RNGFND_MAX_CM'] = 5000
    vehicle.parameters['LAND_FLARE_ALT'] = 2.0
    vehicle.parameters['LAND_FLARE_SEC'] = 2.0
    vehicle.parameters['TECS_LAND_ARSPD'] = 10.0

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
    time.sleep(60)  # Adjust as needed

    print("Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()