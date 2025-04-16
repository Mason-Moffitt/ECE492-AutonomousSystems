from dronekit import connect, VehicleMode, LocationGlobal, Command
from pymavlink import mavutil
import time
import argparse

def wait_for_manual_arm(vehicle):
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle is armed!")

def clear_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()
    print("Cleared any existing mission.")

def add_glide_landing_mission(vehicle, loiter_location, land_location):
    cmds = vehicle.commands
    cmds.clear()

    # MAV_CMD_NAV_LOITER_TO_ALT
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,
        0, 0,
        1, 0, 0, 0,  # loiter radius, unused
        loiter_location.lat,
        loiter_location.lon,
        loiter_location.alt
    ))

    # MAV_CMD_NAV_LAND
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0,
        0, 0, 0, 0,
        land_location.lat,
        land_location.lon,
        land_location.alt
    ))

    cmds.upload()
    print("Landing mission uploaded!")

def main():
    parser = argparse.ArgumentParser(description='Glide land a fixed wing plane.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)

    print("Waiting 10 seconds for takeoff...")
    time.sleep(10)

    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print("Waiting for GUIDED mode...")
        time.sleep(1)

    # Prompt user for landing location
    landing_input = input("Enter landing coordinates as 'lat,lon,alt' (e.g., 35.7721,-78.6745,0): ")
    try:
        lat, lon, alt = map(float, landing_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon,alt")
        return

    # Calculate loiter point ~200m from landing point, 50m up
    loiter_lat = lat + 0.001  # ~110m north
    loiter_lon = lon
    loiter_alt = alt + 50     # 50 meters above landing alt

    loiter_location = LocationGlobal(loiter_lat, loiter_lon, loiter_alt)
    land_location = LocationGlobal(lat, lon, alt)

    clear_mission(vehicle)
    add_glide_landing_mission(vehicle, loiter_location, land_location)

    print("Switching to AUTO mode to begin landing sequence...")
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != "AUTO":
        print("Waiting for AUTO mode...")
        time.sleep(1)

    print("Landing sequence initiated. Monitor via QGroundControl.")
    time.sleep(60)  # Adjust as needed for mission duration

    print("Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()
