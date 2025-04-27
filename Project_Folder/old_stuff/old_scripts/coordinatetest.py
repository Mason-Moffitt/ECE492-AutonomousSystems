from dronekit import connect, VehicleMode, LocationGlobal
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
    while vehicle.mode.name != "TAKEOFF":
        print(" Waiting for mode change...")
        time.sleep(1)
    print("Mode is TAKEOFF. Plane should be launching.")

def get_distance_meters(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt((dlat * 1.113195e5)**2 + (dlon * 1.113195e5)**2)

def add_mission_waypoint(vehicle, lat, lon, alt):
    from dronekit import Command

    print("Clearing previous missions...")
    cmds = vehicle.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()

    print("Creating new mission with 1 waypoint...")
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0,
        0, 0, 0, 0,
        lat, lon, alt
    ))
    cmds.upload()
    print("Mission uploaded.")

def main():
    parser = argparse.ArgumentParser(description='Takeoff and fly to a given GPS coordinate using DroneKit.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

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

    print("Waiting for plane to build speed and altitude...")
    time.sleep(10)

    print("Uploading mission...")
    add_mission_waypoint(vehicle, lat, lon, alt)

    print("Switching to AUTO to follow mission...")
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode.name != "AUTO":
        print(" Waiting for AUTO mode...")
        time.sleep(1)

    target_location = LocationGlobal(lat, lon, alt)

    while True:
        current_location = vehicle.location.global_frame
        distance = get_distance_meters(current_location, target_location)
        print(f"Distance to waypoint: {distance:.1f} meters")
        if distance < 30:  # Looser threshold for fixed-wing
            print("Reached target area!")
            break
        time.sleep(2)

    print("Letting the plane continue or loiter as mission defines...")
    time.sleep(30)

    print("Closing vehicle connection.")
    vehicle.close()

if __name__ == "__main__":
    main()
