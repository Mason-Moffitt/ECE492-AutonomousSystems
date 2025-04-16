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

def upload_single_waypoint(vehicle, lat, lon, alt):
    from dronekit import Command
    cmds = vehicle.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()

    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0,
        0, 0, 0, 0,
        lat, lon, alt
    ))
    cmds.upload()

def upload_loiter(vehicle, lat, lon, alt, radius=50):
    from dronekit import Command
    cmds = vehicle.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()

    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        0, 0,
        radius, 0, 0, 0,
        lat, lon, alt
    ))
    cmds.upload()

def upload_landing(vehicle, lat, lon):
    from dronekit import Command
    cmds = vehicle.commands
    cmds.clear()
    cmds.download()
    cmds.wait_ready()

    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0,
        0, 0, 0, 0,
        lat, lon, 0
    ))
    cmds.upload()

def main():
    parser = argparse.ArgumentParser(description='Fly to waypoint, loiter, prompt for landing point, and land.')
    parser.add_argument('--connect', default='127.0.0.1:14552')
    args = parser.parse_args()

    print("Enter target coordinates as 'lat,lon,alt' (e.g., 35.7721,-78.6745,100): ")
    lat1, lon1, alt1 = map(float, input().split(','))

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)
    set_takeoff_mode(vehicle)

    print("Building speed and altitude...")
    time.sleep(10)

    print("Uploading first waypoint...")
    upload_single_waypoint(vehicle, lat1, lon1, alt1)
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode.name != "AUTO":
        print(" Waiting for AUTO mode...")
        time.sleep(1)

    target1 = LocationGlobal(lat1, lon1, alt1)

    while True:
        current = vehicle.location.global_frame
        distance = get_distance_meters(current, target1)
        print(f"Distance to waypoint: {distance:.1f} m")
        if distance < 40:
            print("Reached first waypoint!")
            break
        time.sleep(2)

    print("Loitering at first location...")
    upload_loiter(vehicle, lat1, lon1, alt1, radius=50)
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode.name != "AUTO":
        time.sleep(1)
    print("Loiter mode active.")

    # Prompt for landing location
    print("Now enter landing coordinates as 'lat,lon' (alt is not needed for LAND):")
    lat2, lon2 = map(float, input().split(','))

    print("Uploading landing command...")
    upload_landing(vehicle, lat2, lon2)
    vehicle.commands.upload()
    time.sleep(2)

    print("Switching to AUTO for landing...")
    vehicle.mode = VehicleMode("AUTO")
    while vehicle.mode.name != "AUTO":
        time.sleep(1)

    target_land = LocationGlobal(lat2, lon2, 0)

    print("Approaching landing target...")
    while True:
        current = vehicle.location.global_frame
        dist = get_distance_meters(current, target_land)
        print(f"Distance to landing point: {dist:.1f} m, Throttle: {vehicle.channels.get('3')}")
        if dist < 10:
            print("Within 10 meters of landing target.")
            break
        time.sleep(2)

    try:
        vehicle.parameters['THR_MIN'] = 0
        print("Throttle set to zero.")
    except Exception as e:
        print(f"Failed to set throttle: {e}")

    print("Waiting for landing detection (optional)...")
    for _ in range(60):
        if vehicle.system_status.state.lower() == 'standby':
            print("Landed and disarmed.")
            break
        time.sleep(1)

    print("Mission complete. Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()
