from dronekit import connect, VehicleMode, LocationGlobal
import time
import argparse
from math import sin, cos, sqrt, atan2, radians, degrees, pi

# Constants
GLIDE_RATIO = 10.0        # Glide ratio (distance/altitude lost)
IDEAL_RADIUS = 100.0      # Distance to start circling (meters)
IDEAL_HEIGHT = 10.0       # Height to stop circling (meters)
IDEAL_SPEED = 10.0        # Speed to stop circling (m/s)
CIRCLE_RADIUS = 50.0      # Desired circling radius (meters)
K_ROLL = 0.1              # Gain for roll control
K_P = 0.1                 # Gain for pitch control
K_DISTANCE = 0.1          # Gain for distance error in bank angle
THROTTLE_MIN = 1000       # PWM value for throttle off
CONTROL_FREQ = 0.05       # 20 Hz control loop
GRAVITY = 9.81            # Gravity (m/s^2)
MAX_ROLL = 45.0           # Max bank angle (degrees)
MAX_PITCH = 5.0           # Max pitch angle (degrees)
MIN_PITCH = -15.0         # Min pitch angle (degrees)

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
    """Send attitude target to control the plane."""
    q = quaternion_from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg))
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0b00000111, q, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)

def set_throttle_off(vehicle):
    """Turn off throttle."""
    vehicle.channels.overrides['3'] = THROTTLE_MIN

def get_distance_metres(location1, location2):
    """Calculate distance between two points in meters."""
    R = 6371000.0
    lat1, lon1 = radians(location1.lat), radians(location1.lon)
    lat2, lon2 = radians(location2.lat), radians(location2.lon)
    dlon, dlat = lon2 - lon1, lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def calculate_bearing(loc1, loc2):
    """Calculate bearing from loc1 to loc2 in degrees."""
    lat1, lon1 = radians(loc1.lat), radians(loc1.lon)
    lat2, lon2 = radians(loc2.lat), radians(loc2.lon)
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    bearing = atan2(x, y)
    return (degrees(bearing) + 360) % 360

def calculate_glide_pitch(glide_ratio):
    """Calculate base pitch for glide ratio."""
    return -degrees(atan2(1, glide_ratio))

def wait_for_manual_arm(vehicle):
    """Wait for the plane to be armed manually."""
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle armed!")

def takeoff_check(vehicle):
    """Confirm takeoff has occurred."""
    print("Waiting for takeoff (altitude > 5m, speed > 5m/s)...")
    while vehicle.location.global_relative_frame.alt < 5.0 or vehicle.airspeed < 5.0:
        time.sleep(1)
    print("Takeoff confirmed.")

def get_waypoint_input(vehicle):
    """Get waypoint coordinates from user."""
    while True:
        try:
            lat, lon = map(float, input("Enter landing coordinates as 'lat,lon': ").split(','))
            waypoint = LocationGlobal(lat, lon, 0)
            current_loc = vehicle.location.global_frame
            current_alt = vehicle.location.global_relative_frame.alt
            distance = get_distance_metres(current_loc, waypoint)
            required_alt = distance / GLIDE_RATIO
            if current_alt < required_alt:
                print(f"Warning: Altitude {current_alt:.1f}m < required {required_alt:.1f}m.")
                if input("Proceed anyway? (y/n): ").lower() == 'y':
                    return waypoint
            else:
                print(f"Waypoint accepted: {distance:.1f}m away.")
                return waypoint
        except ValueError:
            print("Invalid format. Use: lat,lon")

def glide_to_waypoint(vehicle, waypoint):
    """Glide to waypoint with responsive circling."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    print("Mode set to GUIDED. Initiating glide...")

    base_pitch = calculate_glide_pitch(GLIDE_RATIO)
    last_print_time = time.time()

    # Phase 1: Glide towards waypoint
    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading
        ground_speed = vehicle.groundspeed
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity else 0
        distance = get_distance_metres(current_loc, waypoint)

        set_throttle_off(vehicle)

        if distance <= IDEAL_RADIUS:
            print(f"Within {IDEAL_RADIUS}m. Starting circling.")
            break

        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)

        desired_descent_rate = ground_speed / GLIDE_RATIO if ground_speed > 0 else 0
        error_descent = desired_descent_rate - actual_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)

        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)

        if time.time() - last_print_time > 1:
            print(f"Distance: {distance:.1f}m, Alt: {current_alt:.1f}m, Pitch: {desired_pitch:.1f}°")
            last_print_time = time.time()

        time.sleep(CONTROL_FREQ)

    # Phase 2: Circle at CIRCLE_RADIUS
    while current_alt > IDEAL_HEIGHT or current_speed > IDEAL_SPEED:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity else 0
        distance = get_distance_metres(current_loc, waypoint)

        set_throttle_off(vehicle)

        # Bank angle for circular path
        nominal_bank = degrees(atan2(current_speed**2, GRAVITY * CIRCLE_RADIUS))
        error_distance = CIRCLE_RADIUS - distance
        bank_correction = K_DISTANCE * error_distance
        desired_bank = nominal_bank + bank_correction
        desired_bank = max(min(desired_bank, MAX_ROLL), -MAX_ROLL)

        # Pitch control for glide
        desired_descent_rate = current_speed / GLIDE_RATIO if current_speed > 0 else 0
        error_descent = desired_descent_rate - actual_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)

        send_attitude_target(vehicle, desired_bank, desired_pitch, current_yaw)

        if time.time() - last_print_time > 1:
            print(f"Circling - Distance: {distance:.1f}m, Alt: {current_alt:.1f}m, Bank: {desired_bank:.1f}°")
            last_print_time = time.time()

        time.sleep(CONTROL_FREQ)

    # Phase 3: Final approach
    print("Circling complete. Approaching landing...")
    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_yaw = vehicle.heading
        distance = get_distance_metres(current_loc, waypoint)
        vertical_speed = vehicle.velocity[2] if vehicle.velocity else 0

        set_throttle_off(vehicle)

        if current_alt < 0.5 and abs(vertical_speed) < 0.1 and distance < 10.0:
            print("Landing detected. Disarming.")
            vehicle.armed = False
            break

        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)
        desired_pitch = base_pitch
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)
        time.sleep(CONTROL_FREQ)

    print("Landing complete.")
    vehicle.channels.overrides = {}

def main():
    parser = argparse.ArgumentParser(description='Glide to waypoint with circling.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    vehicle = connect(args.connect, wait_ready=['location', 'airspeed'])
    wait_for_manual_arm(vehicle)
    takeoff_check(vehicle)
    waypoint = get_waypoint_input(vehicle)
    glide_to_waypoint(vehicle, waypoint)
    vehicle.close()

if __name__ == "__main__":
    main()
