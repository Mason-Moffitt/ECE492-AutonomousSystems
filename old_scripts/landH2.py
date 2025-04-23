from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time
import argparse

def wait_for_manual_arm(vehicle):
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle is armed!")

def upload_mission(vehicle, takeoff_alt, waypoint_lat, waypoint_lon, land_lat, land_lon):
    cmds = vehicle.commands
    cmds.clear()

    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0,
                     waypoint_lat, waypoint_lon, takeoff_alt))

    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0,
                     waypoint_lat, waypoint_lon, takeoff_alt))

    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                     0, 0, 0, 0, 0, 0,
                     land_lat, land_lon, 0))

    cmds.upload()
    print("Mission uploaded with TAKEOFF, WAYPOINT, and LAND.")

def main():
    parser = argparse.ArgumentParser(description='Takeoff, loiter, and land on user coordinates using a mission.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    # Get loiter location
    loiter_input = input("Enter loiter point (lat,lon,alt): ")
    loiter_lat, loiter_lon, loiter_alt = map(float, loiter_input.split(','))

    # Get landing location
    land_input = input("Enter landing point (lat,lon): ")
    land_lat, land_lon = map(float, land_input.split(','))

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)

    print("Uploading mission...")
    upload_mission(vehicle, loiter_alt, loiter_lat, loiter_lon, land_lat, land_lon)

    print("Switching to AUTO mode to start mission...")
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != "AUTO":
        print(" Waiting for AUTO mode...")
        time.sleep(1)

    print("Mission running. Plane will take off, loiter, then land.")
    time.sleep(60)

    vehicle.close()

if __name__ == "__main__":
    main()
