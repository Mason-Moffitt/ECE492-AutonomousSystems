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

def switch_to_guided(vehicle):
    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print(" Waiting for GUIDED mode...")
        time.sleep(1)
    print("Now in GUIDED mode.")

def fly_to_coordinate(vehicle, lat, lon, alt):
    print(f"Flying to loiter point: lat={lat}, lon={lon}, alt={alt}")
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location)

def send_land_command(vehicle, lat, lon, alt):
    print("Sending NAV_LAND (glide landing) command...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,  # param1-4 (unused)
        lat,
        lon,
        alt
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"Landing command sent to lat={lat}, lon={lon}, alt={alt}")

def main():
    parser = argparse.ArgumentParser(description='Takeoff, loiter, and glide land on user-provided coordinates.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string to vehicle")
    args = parser.parse_args()

    # Prompt user for loiter coordinates
    loiter_input = input("Enter loiter coordinates as 'lat,lon,alt' (SW eg: 35.726010,-78.697287,100): ")
    try:
        loiter_lat, loiter_lon, loiter_alt = map(float, loiter_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon,alt")
        return

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)
    set_takeoff_mode(vehicle)

    print("Waiting for plane to build speed and altitude...")
    time.sleep(10)

    switch_to_guided(vehicle)
    fly_to_coordinate(vehicle, loiter_lat, loiter_lon, loiter_alt)

    print("Loitering... (wait for stable position)")
    time.sleep(15)

    # Prompt user for landing coordinates
    land_input = input("Enter landing coordinates as 'lat,lon,alt'(eg HOME: 35.727312,-78.69610,0): ")
    try:
        land_lat, land_lon, land_alt = map(float, land_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon,alt")
        return

    send_land_command(vehicle, land_lat, land_lon, land_alt)

    print("Monitor landing progress via QGroundControl.")
    time.sleep(30)

    print("Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()
