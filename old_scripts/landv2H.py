from dronekit import connect, VehicleMode, LocationGlobal, Command
from pymavlink import mavutil
import time
import argparse
from math import radians, cos, sin, degrees

def calculate_approach_point(lat, lon, distance_m, bearing_deg):
    R = 6371000  # Earth radius in meters
    lat_rad = radians(lat)
    lon_rad = radians(lon)
    bearing_rad = radians(bearing_deg)
    angular_dist = distance_m / R
    new_lat_rad = lat_rad + cos(bearing_rad) * angular_dist  # Upwind approach
    new_lon_rad = lon_rad + sin(bearing_rad) * angular_dist / cos(lat_rad)
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
    if vehicle.battery.voltage < 10.0:  # Adjust for your battery
        raise ValueError("Battery voltage too low")
    if vehicle._vehicle_type != mavutil.mavlink.MAV_TYPE_FIXED_WING:
        raise ValueError("Vehicle is not a fixed-wing plane")
    print("Pre-flight checks passed.")

def check_landing_parameters(vehicle):
    required_params = {
        'LAND_FLARE_ALT': lambda x: 4 <= x <= 6,  # Flare at 4-6m
        'LAND_FLARE_SEC': lambda x: 2 <= x <= 4,  # 2-4s flare
        'TECS_SINK_MAX': lambda x: 2 <= x <= 3.5,  # 2-3.5 m/s sink rate
        'TRIM_ARSPD_CM': lambda x: 1000 <= x <= 1500  # 10-15 m/s airspeed
    }
    # Wait for parameters to load
    for _ in range(5):  # Retry up to 5 times
        if vehicle.parameters._loaded:
            break
        print("Waiting for parameters to load...")
        time.sleep(1)
    else:
        print("Warning: Parameters not fully loaded. Some checks may fail.")

    for param, check in required_params.items():
        try:
            value = vehicle.parameters[param]
            if not check(value):
                print(f"Warning: {param}={value} may cause landing issues")
            else:
                print(f"{param}={value} is within acceptable range")
        except KeyError:
            print(f"Warning: {param} not found in parameter map")
    print("Landing parameters checked.")

def set_mode(vehicle, mode, timeout=10):
    vehicle.mode = VehicleMode(mode)
    start = time.time()
    while vehicle.mode != mode and time.time() - start < timeout:
        time.sleep(1)
    if vehicle.mode != mode:
        raise RuntimeError(f"Failed to set {mode} mode")

def main():
    parser = argparse.ArgumentParser(description='Takeoff and glide-land for fixed-wing plane.')
    parser.add_argument('--connect', default=':14552', help="Connection string")
    parser.add_argument('--approach-alt', type=float, default=20, help="Approach altitude in meters")
    parser.add_argument('--offset-dist', type=float, default=300, help="Approach offset distance in meters")
    parser.add_argument('--landing-offset', type=float, default=20, help="Landing point offset upwind in meters")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    pre_flight_checks(vehicle)
    check_landing_parameters(vehicle)
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

    heading_input = input("Enter runway heading (degrees, e.g., 180 for south): ")
    try:
        runway_heading = float(heading_input)
    except ValueError:
        print("Invalid heading. Using default 180°")
        runway_heading = 180

    landing_alt = 0
    approach_alt = args.approach_alt
    # Offset landing point upwind by landing-offset meters
    landing_lat, landing_lon = calculate_approach_point(lat, lon, args.landing_offset, runway_heading + 180)
    approach_lat, approach_lon = calculate_approach_point(landing_lat, landing_lon, args.offset_dist, runway_heading + 180)

    cmds = vehicle.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()

    # Start landing sequence
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION,
                     mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    # Set airspeed for landing (10 m/s)
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION,
                     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 1, 10, -1, 0, 0, 0, 0))
    # Approach waypoint
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                     approach_lat, approach_lon, approach_alt))
    # Land
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0,
                     landing_lat, landing_lon, landing_alt))

    cmds.upload()
    print("Landing mission uploaded.")

    print("Switching to AUTO mode for landing...")
    set_mode(vehicle, "AUTO")

    print("Landing in progress...")
    while vehicle.location.global_relative_frame.alt > 1:
        alt = vehicle.location.global_relative_frame.alt
        airspeed = vehicle.airspeed
        groundspeed = vehicle.groundspeed
        pitch = vehicle.attitude.pitch * 180 / 3.14159  # Convert to degrees
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        # Approximate distance to landing point (meters)
        dist = ((landing_lat - curr_lat) * 111139) ** 2 + ((landing_lon - curr_lon) * 111139 * cos(radians(landing_lat))) ** 2
        dist = dist ** 0.5
        print(f"Altitude: {alt:.1f}m, Airspeed: {airspeed:.1f}m/s, Groundspeed: {groundspeed:.1f}m/s, Pitch: {pitch:.1f}°, Distance to landing: {dist:.1f}m")
        time.sleep(1)

    print("Landing complete or near ground level.")
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)
    print("Vehicle disarmed.")
    vehicle.close()

if __name__ == "__main__":
    main()
