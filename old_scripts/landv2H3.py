from dronekit import connect, VehicleMode, LocationGlobal, Command
from pymavlink import mavutil
import time
import argparse

def wait_for_manual_arm(vehicle):
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle is armed!")

def main():
    parser = argparse.ArgumentParser(description='Takeoff and glide-land at a specified waypoint.')
    parser.add_argument('--connect', default=':14552', help="Connection string")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)

    print("Setting mode to TAKEOFF...")
    vehicle.mode = VehicleMode("TAKEOFF")
    while vehicle.mode != "TAKEOFF":
        time.sleep(1)

    print("Waiting for plane to accelerate and climb...")
    time.sleep(10)  # Give it time to build speed

    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)

    # Ask for landing target from user
    coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., 35.7721,-78.6745): ")
    try:
        lat, lon = map(float, coord_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon")
        vehicle.close()
        return

    landing_alt = 0  # Ground altitude for landing
    approach_alt = 30  # Altitude for the approach leg

    # Calculate approach point ~200m behind landing point
    offset = 0.0018  # Roughly ~200m latitude offset
    approach_lat = lat - offset
    approach_lon = lon

    cmds = vehicle.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()

    # DO_LAND_START command (start of landing sequence)
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION,
                     mavutil.mavlink.MAV_CMD_DO_LAND_START,
                     0, 0, 0, 0, 0, 0, 0, 0, 0))
    
    # Set airspeed to 10 m/s for landing using MAVLink
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION,
                     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 1, 10, -1, 0, 0, 0, 0))

    # Approach waypoint
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0,
                     approach_lat, approach_lon, approach_alt))

    # LAND command
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL,
                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                     0, 0, 0, 0, 0, 0,
                     lat, lon, landing_alt))

    cmds.upload()
    print("Landing mission uploaded.")

    # Set mode to AUTO to start mission
    print("Switching to AUTO mode for landing...")
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode != "AUTO":
        time.sleep(1)

    print("Landing in progress. Monitor in QGroundControl.")

    # Let the mission complete
    while vehicle.location.global_relative_frame.alt > 1:
        print(f"Current altitude: {vehicle.location.global_relative_frame.alt:.1f} m")
        time.sleep(2)

    print("Landing complete or near ground level.")
    vehicle.close()

if __name__ == "__main__":
    main()
