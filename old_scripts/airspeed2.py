from dronekit import connect, VehicleMode, LocationGlobal, Command
from pymavlink import mavutil
import time
import argparse
from math import radians, cos, sin, degrees

def calculate_approach_point(lat, lon, distance_m=200, bearing_deg=180):
    R = 6371000
    lat_rad = radians(lat)
    lon_rad = radians(lon)
    bearing_rad = radians(bearing_deg)
    angular_dist = distance_m / R
    new_lat_rad = lat_rad - cos(bearing_rad) * angular_dist
    new_lon_rad = lon_rad - sin(bearing_rad) * angular_dist / cos(lat_rad)
    return degrees(new_lat_rad), degrees(new_lon_rad)

def wait_for_manual_arm(vehicle, timeout=60):
    print("Waiting for manual arming...")
    start = time.time()
    while not vehicle.armed and time.time() - start < timeout:
        time.sleep(1)
    if not vehicle.armed:
        raise RuntimeError("Arming timeout")
    print("Vehicle is armed!")

def pre_flight_checks(vehicle):
    if vehicle.gps_0.fix_type < 3:
        raise ValueError("No 3D GPS fix")
    if vehicle.battery.voltage < 10.0:
        raise ValueError("Battery voltage too low")
    print("Pre-flight checks passed.")

def set_mode(vehicle, mode, timeout=10):
    vehicle.mode = VehicleMode(mode)
    start = time.time()
    while vehicle.mode != mode and time.time() - start < timeout:
        time.sleep(1)
    if vehicle.mode != mode:
        raise RuntimeError(f"Failed to set {mode} mode")

def main():
    parser = argparse.ArgumentParser(description='Takeoff and glide-land for fixed-wing plane.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    parser.add_argument('--approach-alt', type=float, default=30, help="Approach altitude in meters")
    parser.add_argument('--offset-dist', type=float, default=200, help="Approach offset distance in meters")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    pre_flight_checks(vehicle)
    wait_for_manual_arm(vehicle)

    print("Setting mode to TAKEOFF...")
    set_mode(vehicle, "TAKEOFF")
    time.sleep(10)  # Adjust based on TKOFF_ALT

    print("Switching to GUIDED mode...")
    set_mode(vehicle, "GUIDED")

    coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., 35.7721,-78.6745): ")
    try:
        lat, lon = map(float, coord_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon")
        vehicle.close()
        return

    landing_alt = 0
    approach_alt = args.approach_alt
    approach_lat, approach_lon = calculate_approach_point(lat, lon, args.offset_dist)

    cmds = vehicle.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()

    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION,
                     mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    
    # Set throttle to 0% for glide at approach waypoint
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION,
                     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 1, -1, 0, 0, 0, 0, 0))

    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                     approach_lat, approach_lon, approach_alt))
    
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0,
                     lat, lon, landing_alt))

    cmds.upload()
    print("Landing mission uploaded.")

    print("Switching to AUTO mode for landing...")
    set_mode(vehicle, "AUTO")

    print("Landing in progress...")
    while vehicle.location.global_relative_frame.alt > 1:
        alt = vehicle.location.global_relative_frame.alt
        airspeed = vehicle.airspeed
        print(f"Altitude: {alt:.1f}m, Airspeed: {airspeed:.1f}m/s")
        time.sleep(2)

    print("Landing complete or near ground level.")
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)
    print("Vehicle disarmed.")
    vehicle.close()

if __name__ == "__main__":
    main()
