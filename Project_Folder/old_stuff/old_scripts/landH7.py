from dronekit import connect, VehicleMode, LocationGlobalRelative
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

def fly_to_coordinate(vehicle, lat, lon, alt):
    print(f"Flying to target: lat={lat}, lon={lon}, alt={alt}")
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location)

def loiter_at_altitude(vehicle, altitude):
    print(f"Loitering at altitude {altitude} meters...")
    location = LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, altitude)
    vehicle.simple_goto(location)
    time.sleep(20)  # Time to loiter at the altitude, can adjust based on use case

def land_plane(vehicle):
    print("Initiating landing procedure...")
    # Switch to RTL mode for return-to-launch and landing behavior
    vehicle.mode = VehicleMode("RTL")
    while vehicle.mode != "RTL":
        print(" Waiting for mode change to RTL...")
        time.sleep(1)
    print("Landing initiated in RTL mode.")
    
    # You can monitor the altitude and ensure the plane is descending properly
    while vehicle.location.global_relative_frame.alt > 10:  # Example threshold to stop waiting
        print(f"Plane altitude: {vehicle.location.global_relative_frame.alt} meters")
        time.sleep(1)
    print("Plane has landed!")

def main():
    parser = argparse.ArgumentParser(description='Takeoff, fly, and land the plane at a user-defined waypoint.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)
    set_takeoff_mode(vehicle)

    # Wait for plane to take off and climb to safe altitude
    print("Waiting for plane to build speed and altitude...")
    time.sleep(10)

    # Set plane to GUIDED mode for waypoint navigation
    print("Switching to GUIDED mode for waypoint navigation...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        print(" Waiting for GUIDED mode...")
        time.sleep(1)

    # Takeoff and fly to loiter altitude (e.g., 100m)
    loiter_altitude = 100  # Set your desired loiter altitude
    loiter_at_altitude(vehicle, loiter_altitude)

    # Now ask for landing waypoint after loitering
    coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., 35.7721,-78.6745): ")
    try:
        lat, lon = map(float, coord_input.split(','))
    except ValueError:
        print("Invalid format. Use: lat,lon")
        return

    print(f"Flying to landing coordinates: lat={lat}, lon={lon}")
    fly_to_coordinate(vehicle, lat, lon, loiter_altitude)  # Fly to the user-defined landing position

    # Wait to ensure the plane reaches the waypoint before landing
    print("Plane has reached the waypoint. Initiating landing...")
    time.sleep(10)

    # Switch to RTL mode and make sure the plane is descending
    print("Switching to RTL mode and gliding to the landing point...")
    land_plane(vehicle)
    
    # Close the vehicle connection
    print("Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()
