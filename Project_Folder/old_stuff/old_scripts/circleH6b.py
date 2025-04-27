from dronekit import connect, VehicleMode, LocationGlobal
import time
import argparse
from math import sin, cos, sqrt, atan2, radians, degrees, pi

# Constants
GLIDE_RATIO = 10.0        # Tune this in SITL: Test glide in SITL, measure distance/altitude lost (e.g., 1000m/100m = 10.0)
IDEAL_RADIUS = 100.0      # Distance to start circling (meters)
IDEAL_HEIGHT = 10.0       # Target height to stop circling (meters)
IDEAL_SPEED = 10.0        # Target speed to stop circling (m/s)
CIRCLE_RADIUS = 50.0      # Radius for circling (meters)
K_ROLL = 0.1              # Proportional gain for roll
THROTTLE_MIN = 1000       # PWM value for throttle off
CONTROL_FREQ = 0.05       # Control loop frequency (20 Hz)
GRAVITY = 9.81            # Acceleration due to gravity (m/s^2)

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    return q

def send_attitude_target(vehicle, roll_deg, pitch_deg, yaw_deg):
    """Send SET_ATTITUDE_TARGET message to control attitude."""
    q = quaternion_from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg))
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        0,  # target_system
        0,  # target_component
        0b00000111,  # type_mask: ignore rates, use attitude
        q,  # quaternion
        0,  # body roll rate
        0,  # body pitch rate
        0,  # body yaw rate
        0   # thrust (not used, relying on RC override)
    )
    vehicle.send_mavlink(msg)

def set_throttle_off(vehicle):
    """Persistently set throttle to minimum via RC override."""
    vehicle.channels.overrides['3'] = THROTTLE_MIN

def get_distance_metres(location1, location2):
    """Calculate distance between two LocationGlobal points in meters."""
    R = 6371000.0  # Earth's radius in meters
    lat1 = radians(location1.lat)
    lon1 = radians(location1.lon)
    lat2 = radians(location2.lat)
    lon2 = radians(location2.lon)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def calculate_bearing(loc1, loc2):
    """Calculate bearing from loc1 to loc2 in degrees."""
    lat1 = radians(loc1.lat)
    lon1 = radians(loc1.lon)
    lat2 = radians(loc2.lat)
    lon2 = radians(loc2.lon)
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    bearing = atan2(x, y)
    return (degrees(bearing) + 360) % 360

def calculate_glide_pitch(glide_ratio):
    """Calculate pitch angle to maintain glide ratio."""
    return -degrees(atan2(1, glide_ratio))

def wait_for_manual_arm(vehicle):
    """Wait for manual arming."""
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle armed!")

def takeoff_check(vehicle):
    """Confirm takeoff."""
    print("Waiting for takeoff (altitude > 5m, speed > 5m/s)...")
    while vehicle.location.global_relative_frame.alt < 5.0 or vehicle.airspeed < 5.0:
        time.sleep(1)
    print("Takeoff confirmed.")

def get_waypoint_input(vehicle):
    """Get and validate waypoint, checking if current altitude is sufficient."""
    while True:
        coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., 35.7721,-78.6745): ")
        try:
            lat, lon = map(float, coord_input.split(','))
            waypoint = LocationGlobal(lat, lon, 0)
            current_loc = vehicle.location.global_frame
            current_alt = vehicle.location.global_relative_frame.alt
            distance = get_distance_metres(current_loc, waypoint)
            required_alt = distance / GLIDE_RATIO
            if current_alt < required_alt:
                print(f"Warning: Current altitude {current_alt:.1f}m is less than required {required_alt:.1f}m to reach waypoint.")
                choice = input("Do you want to (1) choose a new waypoint or (2) proceed anyway? Enter 1 or 2: ")
                if choice == '1':
                    continue
                elif choice == '2':
                    print("Proceeding with insufficient altitude. Plane may not reach waypoint.")
                    return waypoint
                else:
                    print("Invalid choice. Please enter 1 or 2.")
                    continue
            else:
                print(f"Waypoint accepted: {distance:.1f}m away, current altitude {current_alt:.1f}m >= required {required_alt:.1f}m.")
                return waypoint
        except ValueError:
            print("Invalid format. Use: lat,lon")

def glide_to_waypoint(vehicle, waypoint):
    """Glide to waypoint with throttle off, using distance, height, and speed."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    print("Mode set to GUIDED. Initiating glide with throttle off...")

    # Calculate pitch for desired glide ratio
    desired_pitch = calculate_glide_pitch(GLIDE_RATIO)

    # Initial glide towards waypoint
    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading
        distance = get_distance_metres(current_loc, waypoint)

        set_throttle_off(vehicle)

        if distance <= IDEAL_RADIUS:
            print(f"Within {IDEAL_RADIUS}m of waypoint. Starting circling.")
            break

        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)
        time.sleep(CONTROL_FREQ)

    # Circling phase with fixed bank angle
    while current_alt > IDEAL_HEIGHT or current_speed > IDEAL_SPEED:
        set_throttle_off(vehicle)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading

        # Calculate bank angle for circular flight
        desired_bank = degrees(atan2(current_speed**2, GRAVITY * CIRCLE_RADIUS))
        send_attitude_target(vehicle, desired_bank, desired_pitch, current_yaw)
        time.sleep(CONTROL_FREQ)

    # Final approach
    print("Circling complete. Approaching waypoint for landing...")
    while True:
        set_throttle_off(vehicle)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_yaw = vehicle.heading
        distance = get_distance_metres(current_loc, waypoint)
        vertical_speed = vehicle.velocity[2] if vehicle.velocity else 0

        if current_alt < 0.5 and abs(vertical_speed) < 0.1 and distance < 10.0:
            print("Landing detected. Disarming vehicle.")
            vehicle.armed = False
            break

        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)
        time.sleep(CONTROL_FREQ)

    print("Landing complete.")
    vehicle.channels.overrides = {}  # Clear overrides

def main():
    parser = argparse.ArgumentParser(description='Glide to waypoint with throttle off.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=['location', 'airspeed'])

    wait_for_manual_arm(vehicle)
    takeoff_check(vehicle)
    waypoint = get_waypoint_input(vehicle)
    glide_to_waypoint(vehicle, waypoint)

    vehicle.close()

if __name__ == "__main__":
    main()
