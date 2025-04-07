import time
from dronekit import connect, VehicleMode

# Connect to the vehicle (replace the IP/port if needed)
connection_string = 'tcp:127.0.0.1:5760'  # Make sure this matches your setup
vehicle = connect(connection_string, wait_ready=True)

# Wait for the vehicle to initialize
print("Waiting for vehicle to initialize...")
while not vehicle.is_armable:
    print("Waiting for vehicle to become armable...")
    time.sleep(1)

# Arm the vehicle
print("Arming vehicle...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.arm()

# Wait for the vehicle to arm
while not vehicle.armed:
    print("Waiting for vehicle to arm...")
    time.sleep(1)

# Set the mode to TAKEOFF (altitude you want to reach, e.g., 10 meters)
takeoff_altitude = 10
vehicle.simple_takeoff(takeoff_altitude)

# Wait until the plane reaches the desired altitude
while True:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt} meters")
    if vehicle.location.global_relative_frame.alt >= takeoff_altitude * 0.95:
        print("Takeoff complete")
        break
    time.sleep(1)

# Optionally: Change mode to "AUTOPILOT" to follow the flight plan or another mission
vehicle.mode = VehicleMode("AUTOPILOT")

# Close the connection
vehicle.close()
