from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

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

def set_loiter_radius(vehicle, radius):
    print(f"Setting loiter radius to {radius} meters...")
    vehicle.parameters['WP_LOITER_RAD'] = radius
    while abs(vehicle.parameters['WP_LOITER_RAD'] - radius) > 0.1:  # Check if set correctly
        print(" Waiting for parameter update...")
        time.sleep(1)
    print(f"Loiter radius set to {vehicle.parameters['WP_LOITER_RAD']} meters.")

def fly_to_coordinate(vehicle, lat, lon, alt):
    print(f"Flying to target: lat={lat}, lon={lon}, alt={alt}")
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location, airspeed=10)

def main():
    parser = argparse.ArgumentParser(description='Takeoff and fly to a given GPS coordinate.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    # Prompt user for coordinates
    coord_input = input("Enter target coordinates as 'lat,lon,alt' (e.g., 35.7721,-78.6745,100): ")
    try:
        lat, lon, alt = map(float, coord_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon,alt")
        return

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)
    set_takeoff_mode(vehicle)

    # Wait for plane to be airborne
    print("Waiting for plane to build speed and altitude...")
    time.sleep(20)

    # Switch to GUIDED before sending waypoint
    print("Switching to AUTO mode for waypoint navigation...")
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != "AUTO":
        print(" Waiting for AUTO mode...")
        time.sleep(1)

    # Fly to the user-input location
    fly_to_coordinate(vehicle, lat, lon, alt)
    time.sleep(3)  # Wait for it to reach the location
    
    print("Setting throttle to 0%")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 3, -1, 1, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

    # Wait for landing to complete (arbitrary time for demonstration)
    time.sleep(30)

    print("Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()