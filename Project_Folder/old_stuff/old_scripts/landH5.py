from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import argparse
import math

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

def fly_to_coordinate(vehicle, lat, lon, alt):
    print(f"Flying to loiter point: lat={lat}, lon={lon}, alt={alt}")
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location)

def add_approach_waypoint(lat, lon, bearing, distance):
    """Compute a point 'distance' meters behind lat/lon based on bearing (degrees)."""
    earth_radius = 6378137.0  # Radius of Earth in meters

    bearing_rad = math.radians(bearing)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    new_lat = math.asin(math.sin(lat_rad) * math.cos(distance / earth_radius) -
                        math.cos(lat_rad) * math.sin(distance / earth_radius) * math.cos(bearing_rad))

    new_lon = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / earth_radius) * math.cos(lat_rad),
                                   math.cos(distance / earth_radius) - math.sin(lat_rad) * math.sin(new_lat))

    return math.degrees(new_lat), math.degrees(new_lon)

def send_landing_mission(vehicle, land_lat, land_lon, land_alt):
    cmds = vehicle.commands
    cmds.clear()

    # Calculate approach waypoint ~170 meters before landing, opposite of bearing
    bearing = 0  # You can ask the user or assume north
    approach_lat, approach_lon = add_approach_waypoint(land_lat, land_lon, bearing, 170)

    print(f"Adding approach waypoint at {approach_lat}, {approach_lon}")

    cmds.add(Command(0, 0, 0,
                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0,
                     approach_lat, approach_lon, land_alt))

    # LAND command
    cmds.add(Command(0, 0, 0,
                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                     0, 0, 0, 0, 0, 0,
                     land_lat, land_lon, 0))

    cmds.upload()

    print("Landing mission uploaded. Switching to AUTO mode...")
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != "AUTO":
        print(" Waiting for AUTO mode...")
        time.sleep(1)

    # Cut throttle to glide
    print("Setting throttle to 0 for glide landing...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 3, -1, 1, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)

def main():
    parser = argparse.ArgumentParser(description='Glide land at specified waypoint.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    # Prompt user for landing coordinates
    coord_input = input("Enter landing coordinates as 'lat,lon,alt' (e.g., 35.7721,-78.6745,50): ")
    try:
        land_lat, land_lon, land_alt = map(float, coord_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon,alt")
        return

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)
    set_takeoff_mode(vehicle)

    print("Waiting for plane to build speed and altitude...")
    time.sleep(10)

    print("Switching to GUIDED mode for loiter navigation...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)

    # Loiter near landing target before actual landing
    fly_to_coordinate(vehicle, land_lat, land_lon, land_alt)

    print("Loitering near landing point...")
    time.sleep(20)

    send_landing_mission(vehicle, land_lat, land_lon, land_alt)

    print("Landing sequence started. Monitor via QGroundControl.")

    time.sleep(60)
    vehicle.close()

if __name__ == "__main__":
    main()
