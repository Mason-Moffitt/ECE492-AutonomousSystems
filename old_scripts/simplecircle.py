from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import argparse
from math import sin, cos, sqrt, atan2, radians, degrees, pi

# Configurable Constants
GLIDE_RATIO = 10.0        # Initial glide ratio (distance/altitude), tune in SITL
IDEAL_RADIUS = 100.0      # Distance to start circling (meters)
IDEAL_HEIGHT = 10.0       # Height to stop circling (meters)
IDEAL_SPEED = 10.0        # Speed to stop circling (m/s)
CIRCLE_RADIUS = 100.0     # Desired circling radius (meters)
K_ROLL = 0.6              # Roll proportional gain
K_P = 0.1                 # Pitch proportional gain
K_DISTANCE = 0.4          # Distance correction gain for bank angle (conservative)
THROTTLE_MIN = 1000       # PWM value for throttle off
CONTROL_FREQ = 0.05       # Control loop frequency (20 Hz)
GRAVITY = 9.81            # Gravity (m/s^2)
MAX_ROLL = 30.0           # Max roll angle (degrees)
MAX_PITCH = 5.0           # Max pitch angle (degrees)
MIN_PITCH = -15.0         # Min pitch angle (degrees)
MIN_TAKEOFF_ALT = 5.0     # Minimum altitude for takeoff check (meters)
MIN_START_ALT = 5.0       # Minimum starting altitude (meters)
HORIZONTAL_TOLERANCE = 35.0  # Horizontal distance to consider start location reached (meters)
VERTICAL_TOLERANCE = 5.0    # Vertical distance to consider start altitude reached (meters)

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion for attitude control."""
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
    """Send attitude control commands via MAVLink."""
    q = quaternion_from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg))
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0b00000111, q, 0, 0, 0, 0  # Ignore rates, use attitude
    )
    vehicle.send_mavlink(msg)

def set_throttle_off(vehicle):
    """Set throttle to minimum using RC override."""
    vehicle.channels.overrides['3'] = THROTTLE_MIN

def get_distance_metres(location1, location2):
    """Calculate distance between two global locations."""
    R = 6371000.0  # Earth's radius (meters)
    lat1, lon1 = radians(location1.lat), radians(location1.lon)
    lat2, lon2 = radians(location2.lat), radians(location2.lon)
    dlon, dlat = lon2 - lon1, lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def calculate_bearing(loc1, loc2):
    """Calculate bearing from loc1 to loc2."""
    lat1, lon1 = radians(loc1.lat), radians(loc1.lon)
    lat2, lon2 = radians(loc2.lat), radians(loc2.lon)
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    bearing = atan2(x, y)
    return (degrees(bearing) + 360) % 360

def calculate_glide_pitch(glide_ratio):
    """Calculate base pitch for desired glide ratio."""
    return -degrees(atan2(1, glide_ratio))

def wait_for_manual_arm(vehicle):
    """Wait for the vehicle to be armed manually."""
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle armed!")

def takeoff_check(vehicle):
    """Wait for takeoff confirmation."""
    print(f"Waiting for takeoff (altitude > {MIN_TAKEOFF_ALT}m, speed > 5m/s)...")
    while (vehicle.location.global_relative_frame.alt < MIN_TAKEOFF_ALT or 
           vehicle.airspeed < 5.0):
        time.sleep(1)
    print("Takeoff confirmed.")

def get_start_location_input():
    """Get starting location (lat, lon, alt) from user."""
    while True:
        try:
            coord_input = input("Enter starting location as 'lat,lon,alt' (e.g.SW, 35.726010,-78.697287,100): ")
            lat, lon, alt = map(float, coord_input.split(','))
            if alt < MIN_START_ALT:
                print(f"Altitude must be at least {MIN_START_ALT}m.")
                continue
            return LocationGlobalRelative(lat, lon, alt)
        except ValueError:
            print("Invalid format. Use: lat,lon,alt")

def navigate_to_start_location(vehicle, start_location):
    """Navigate to the starting location."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    print(f"Navigating to starting location: {start_location.lat}, {start_location.lon}, {start_location.alt}m...")

    vehicle.simple_goto(start_location)
    
    # Wait until the plane is close to the target location
    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        distance = get_distance_metres(current_loc, start_location)
        alt_diff = abs(current_alt - start_location.alt)
        
        if distance < HORIZONTAL_TOLERANCE and alt_diff < VERTICAL_TOLERANCE:
            print("Starting location reached.")
            break
        
        print(f"Distance to start: {distance:.1f}m, Altitude difference: {alt_diff:.1f}m")
        time.sleep(1)

def get_waypoint_input(vehicle):
    """Get landing waypoint from user and validate altitude."""
    while True:
        try:
            coord_input = input("Enter landing coordinates as 'lat,lon' (e.g. HOME: 35.727312, -78.696101): ")
            lat, lon = map(float, coord_input.split(','))
            waypoint = LocationGlobal(lat, lon, 0)
            current_loc = vehicle.location.global_frame
            current_alt = vehicle.location.global_relative_frame.alt
            distance = get_distance_metres(current_loc, waypoint)
            required_alt = distance / GLIDE_RATIO
            if current_alt < required_alt:
                print(f"Warning: Altitude {current_alt:.1f}m < required {required_alt:.1f}m.")
                choice = input("1) New waypoint, 2) Proceed anyway? Enter 1 or 2: ")
                if choice == '1':
                    continue
                elif choice == '2':
                    return waypoint
            else:
                print(f"Waypoint accepted: {distance:.1f}m away.")
                return waypoint
        except ValueError:
            print("Invalid format. Use: lat,lon")

def glide_to_waypoint(vehicle, waypoint):
    """Control glide and circling to waypoint using simplified bank angle control."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    print("Mode set to GUIDED. Starting glide...")

    base_pitch = calculate_glide_pitch(GLIDE_RATIO)
    last_print_time = time.time()
    last_bank = 0.0       # For smooth bank transition

    # Glide towards waypoint
    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading
        ground_speed = vehicle.groundspeed
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
        distance = get_distance_metres(current_loc, waypoint)

        set_throttle_off(vehicle)

        if distance <= IDEAL_RADIUS:
            # Determine circling direction based on heading error
            bearing = calculate_bearing(current_loc, waypoint)
            heading_error = (bearing - current_yaw + 180) % 360 - 180
            circle_direction = 'CW' if heading_error >= 0 else 'CCW'
            direction_multiplier = 1 if circle_direction == 'CW' else -1
            print(f"Within {IDEAL_RADIUS}m. Starting {circle_direction} circling.")
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

    # Circling phase
    with open('circling_log.txt', 'a') as f:
        f.write("Time,Distance,Bank,Error_Distance,Speed\n")
    while current_alt > IDEAL_HEIGHT or current_speed > IDEAL_SPEED:
        set_throttle_off(vehicle)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
        distance = get_distance_metres(current_loc, waypoint)

        # Calculate nominal bank angle
        nominal_bank = degrees(atan2(current_speed**2, GRAVITY * CIRCLE_RADIUS))
        nominal_bank = min(nominal_bank, MAX_ROLL - 5.0)  # Cap to leave room for corrections

        # Proportional distance error (inverted sign)
        error_distance = CIRCLE_RADIUS - distance
        bank_correction = -K_DISTANCE * error_distance  # Negative sign to correct direction

        # Apply direction multiplier
        desired_bank = (nominal_bank + bank_correction) * direction_multiplier
        desired_bank = max(min(desired_bank, MAX_ROLL), -MAX_ROLL)

        # Smooth bank transition
        desired_bank = 0.95 * last_bank + 0.1 * desired_bank
        last_bank = desired_bank

        desired_descent_rate = current_speed / GLIDE_RATIO if current_speed > 0 else 0
        error_descent = desired_descent_rate - actual_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)

        send_attitude_target(vehicle, desired_bank, desired_pitch, current_yaw)

        # Log data for analysis
        with open('circling_log.txt', 'a') as f:
            f.write(f"{time.time()},{distance},{desired_bank},{error_distance},{current_speed}\n")

        if time.time() - last_print_time > 1:
            print(f"Circling - Distance: {distance:.1f}m, Alt: {current_alt:.1f}m, Bank: {desired_bank:.1f}°")
            last_print_time = time.time()

        time.sleep(CONTROL_FREQ)

    # Final approach
    print("Circling complete. Approaching waypoint...")
    while True:
        set_throttle_off(vehicle)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_yaw = vehicle.heading
        distance = get_distance_metres(current_loc, waypoint)
        vertical_speed = vehicle.velocity[2] if vehicle.velocity is not None else 0

        if current_alt < 2.2 and abs(vertical_speed) < 0.1:
            final_distance = get_distance_metres(current_loc, waypoint)
            print("Landing detected.")
            vehicle.mode = VehicleMode("MANUAL")
            while vehicle.mode != "MANUAL":
                time.sleep(1)
            print("Mode set to MANUAL. Ready to disarm...")
            vehicle.armed = False
            print(f"Final distance from waypoint: {final_distance:.1f} meters")
            break

        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)
        desired_pitch = base_pitch
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)

        if time.time() - last_print_time > 1:
            print(f"Distance: {distance:.1f}m, Alt: {current_alt:.1f}m, Pitch: {desired_pitch:.1f}°")
            last_print_time = time.time()

        time.sleep(CONTROL_FREQ)

    print("Landing complete.")
    vehicle.channels.overrides = {}

def main():
    parser = argparse.ArgumentParser(description='Glide to waypoint with throttle off.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=['location', 'airspeed'])

    wait_for_manual_arm(vehicle)
    takeoff_check(vehicle)
    start_location = get_start_location_input()
    navigate_to_start_location(vehicle, start_location)
    waypoint = get_waypoint_input(vehicle)
    glide_to_waypoint(vehicle, waypoint)

    vehicle.close()

if __name__ == "__main__":
    main()
