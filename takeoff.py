from dronekit import connect, VehicleMode
import time

print("connecting to udp")
# Connect to the Vehicle (simulator)
vehicle = connect('tcp:127.0.0.1:14551', wait_ready=True)
print("connected to udp")

# Function to arm the vehicle
def arm_vehicle():
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    print("Arming vehicle...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.arm()

    while not vehicle.armed:
        print(" Waiting for vehicle to arm...")
        time.sleep(1)
    
    print("Vehicle armed!")

# Function to take off
def takeoff(altitude):
    print("Taking off!")
    vehicle.simple_takeoff(altitude)

    # Wait until the plane reaches the target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude!")
            break
        time.sleep(1)

# Main script
arm_vehicle()
takeoff(10)  # Target altitude in meters

# Close the vehicle connection after the task is complete
vehicle.close()
