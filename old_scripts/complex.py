from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import argparse
import logging
from math import sin, cos, sqrt, atan2, radians, degrees, pi

# Configuration Constants
GLIDE_RATIO = 10.0        # Initial glide ratio (distance/altitude)
IDEAL_RADIUS = 100.0       # Distance to start circling (meters)
IDEAL_HEIGHT = 10.0       # Height to stop circling (meters)
SAFE_AIRSPEED = 7.0       # Minimum airspeed to prevent stall (m/s)
CIRCLE_RADIUS = 75.0      # Desired circling radius (meters)
K_ROLL = 1.0              # Roll proportional gain for glide and final approach
K_P = 0.1                 # Pitch proportional gain
K_HEADING = 0.9           # Heading error gain for circling
K_DISTANCE = 0.1          # Distance correction gain for circling
K_WIND = 0.05             # Wind correction gain
THROTTLE_MIN = 1000       # PWM value for throttle off
CONTROL_FREQ = 0.05       # Control loop frequency (20 Hz)
MAX_ROLL = 30.0           # Max roll angle (degrees)
MAX_PITCH = 5.0           # Max pitch angle (degrees)
MIN_PITCH = -15.0         # Min pitch angle (degrees)
MIN_TAKEOFF_ALT = 5.0     # Minimum altitude for takeoff check (meters)
MIN_START_ALT = 5.0       # Minimum starting altitude (meters)
HORIZONTAL_TOLERANCE = 45.0  # Horizontal tolerance for start location (meters)
VERTICAL_TOLERANCE = 5.0    # Vertical tolerance for start altitude (meters)
K_HEADING_INITIAL = 1.5   # Aggressive heading gain for initial circling
MAX_CIRCLE_TIME = 300     # Max circling time (seconds)
g = 9.81                  # Gravity (m/s^2)

# Setup logging
logging.basicConfig(filename='flight_log.log', level=logging.INFO,
                    format='%(asctime)s %(message)s')

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
        0, 0, 0, 0b00000111, q, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)

def set_throttle_off(vehicle):
    """Set throttle to minimum using RC override."""
    vehicle.channels.overrides['3'] = THROTTLE_MIN

def get_distance_metres(location1, location2):
    """Calculate distance between two global locations."""
    R = 6371000.0
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
    logging.info("Waiting for manual arming...")
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    logging.info("Vehicle armed!")
    print("Vehicle armed!")

def takeoff_check(vehicle):
    """Wait for takeoff confirmation."""
    logging.info(f"Waiting for takeoff (altitude > {MIN_TAKEOFF_ALT}m, speed > 5m/s)...")
    print(f"Waiting for takeoff (altitude > {MIN_TAKEOFF_ALT}m, speed > 5m/s)...")
    while (vehicle.location.global_relative_frame.alt < MIN_TAKEOFF_ALT or
           vehicle.airspeed < 5.0):
        time.sleep(1)
    logging.info("Takeoff confirmed.")
    print("Takeoff confirmed.")

def get_start_location_input():
    """Get starting location (lat, lon, alt) from user with validation."""
    while True:
        try:
            coord_input = input("Enter starting location as 'lat,lon,alt' (e.g., 35.726010,-78.697287,100): ")
            lat, lon, alt = map(float, coord_input.split(','))
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                print("Invalid coordinates. Latitude: [-90, 90], Longitude: [-180, 180].")
                continue
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
    logging.info(f"Navigating to start: {start_location.lat}, {start_location.lon}, {start_location.alt}m...")
    print(f"Navigating to start: {start_location.lat}, {start_location.lon}, {start_location.alt}m...")
    vehicle.simple_goto(start_location)

    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt]):
            print("Invalid sensor data. Retrying...")
            time.sleep(1)
            continue
        distance = get_distance_metres(current_loc, start_location)
        alt_diff = abs(current_alt - start_location.alt)
        if distance < HORIZONTAL_TOLERANCE and alt_diff < VERTICAL_TOLERANCE:
            logging.info("Starting location reached.")
            print("Starting location reached.")
            break
        logging.info(f"Distance to start: {distance:.1f}m, Altitude diff: {alt_diff:.1f}m")
        print(f"Distance to start: {distance:.1f}m, Altitude diff: {alt_diff:.1f}m")
        time.sleep(1)

def get_waypoint_input(vehicle):
    """Get landing waypoint from user and validate altitude."""
    while True:
        try:
            coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., 35.727312,-78.696101): ")
            lat, lon = map(float, coord_input.split(','))
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                print("Invalid coordinates. Latitude: [-90, 90], Longitude: [-180, 180].")
                continue
            waypoint = LocationGlobal(lat, lon, 0)
            current_loc = vehicle.location.global_frame
            current_alt = vehicle.location.global_relative_frame.alt
            if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt]):
                print("Invalid sensor data. Retrying...")
                continue
            distance = get_distance_metres(current_loc, waypoint)
            required_alt = distance / GLIDE_RATIO
            if current_alt < required_alt:
                logging.warning(f"Altitude {current_alt:.1f}m < required {required_alt:.1f}m.")
                print(f"Warning: Altitude {current_alt:.1f}m < required {required_alt:.1f}m.")
                choice = input("1) New waypoint, 2) Proceed anyway? Enter 1 or 2: ")
                if choice == '1':
                    continue
                elif choice == '2':
                    return waypoint
            logging.info(f"Waypoint accepted: {distance:.1f}m away.")
            print(f"Waypoint accepted: {distance:.1f}m away.")
            return waypoint
        except ValueError:
            print("Invalid format. Use: lat,lon")

def glide_to_waypoint(vehicle, waypoint):
    """Guide drone to waypoint with throttle off, including circling phase."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    logging.info("Mode set to GUIDED. Starting glide...")
    print("Mode set to GUIDED. Starting glide...")

    base_pitch = calculate_glide_pitch(GLIDE_RATIO)
    last_print_time = time.time()
    last_bank = 0.0
    last_descent_rate = 0.0
    alpha = 0.8  # Exponential filter coefficient

    # Glide towards waypoint
    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading
        ground_speed = vehicle.groundspeed
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
        if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt, current_speed]):
            print("Invalid sensor data. Skipping cycle.")
            time.sleep(CONTROL_FREQ)
            continue
        distance = get_distance_metres(current_loc, waypoint)
        set_throttle_off(vehicle)

        if distance <= IDEAL_RADIUS:
            logging.info(f"Within {IDEAL_RADIUS}m. Starting CCW circling...")
            print(f"Within {IDEAL_RADIUS}m. Starting CCW circling...")
            direction_multiplier = -1  # CCW (left turn)

            # Initial circling: Align heading with tangent
            while True:
                set_throttle_off(vehicle)
                current_loc = vehicle.location.global_frame
                current_alt = vehicle.location.global_relative_frame.alt
                current_speed = vehicle.airspeed
                current_yaw = vehicle.heading
                actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
                if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt, current_speed]):
                    print("Invalid sensor data. Skipping cycle.")
                    time.sleep(CONTROL_FREQ)
                    continue
                distance = get_distance_metres(current_loc, waypoint)
                bearing_to_plane = calculate_bearing(waypoint, current_loc)
                tangent_direction = (bearing_to_plane + 90) % 360  # CCW tangent
                heading_error = (tangent_direction - current_yaw + 180) % 360 - 180

                if abs(heading_error) < 5.0:
                    break

                k_heading_current = K_HEADING_INITIAL if current_speed > SAFE_AIRSPEED else K_HEADING
                desired_bank = k_heading_current * heading_error * direction_multiplier
                desired_bank = max(min(desired_bank, MAX_ROLL), -MAX_ROLL)
                desired_bank = 0.7 * last_bank + 0.3 * desired_bank
                last_bank = desired_bank

                desired_descent_rate = current_speed / GLIDE_RATIO if current_speed > 0 else 0
                filtered_descent_rate = alpha * last_descent_rate + (1 - alpha) * actual_descent_rate
                last_descent_rate = filtered_descent_rate
                error_descent = desired_descent_rate - filtered_descent_rate
                pitch_adjustment = K_P * error_descent
                desired_pitch = base_pitch + pitch_adjustment
                desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)

                send_attitude_target(vehicle, desired_bank, desired_pitch, current_yaw)

                if time.time() - last_print_time > 0.5:
                    logging.info(f"Initial circling - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Bank: {desired_bank:.1f}°, Error: {heading_error:.1f}°")
                    print(f"Initial circling - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Bank: {desired_bank:.1f}°, Error: {heading_error:.1f}°")
                    last_print_time = time.time()
                time.sleep(CONTROL_FREQ)

            logging.info("Heading aligned. Entering normal circling.")
            print("Heading aligned. Entering normal circling.")
            break

        # Glide phase control
        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)

        desired_descent_rate = ground_speed / GLIDE_RATIO if ground_speed > 0 else 0
        filtered_descent_rate = alpha * last_descent_rate + (1 - alpha) * actual_descent_rate
        last_descent_rate = filtered_descent_rate
        error_descent = desired_descent_rate - filtered_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)

        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)

        if time.time() - last_print_time > 1:
            logging.info(f"Glide - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Pitch: {desired_pitch:.1f}°")
            print(f"Glide - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Pitch: {desired_pitch:.1f}°")
            last_print_time = time.time()
        time.sleep(CONTROL_FREQ)

    # Normal circling phase
    circle_start_time = time.time()
    while (current_alt > IDEAL_HEIGHT and current_speed > SAFE_AIRSPEED and
           time.time() - circle_start_time < MAX_CIRCLE_TIME):
        set_throttle_off(vehicle)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        current_yaw = vehicle.heading
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
        if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt, current_speed]):
            print("Invalid sensor data. Skipping cycle.")
            time.sleep(CONTROL_FREQ)
            continue
        distance = get_distance_metres(current_loc, waypoint)
        bearing_to_plane = calculate_bearing(waypoint, current_loc)
        tangent_direction = (bearing_to_plane + 90) % 360  # CCW

        # Simplified bank calculation: Focus on heading and distance
        heading_error = (tangent_direction - current_yaw + 180) % 360 - 180
        error_distance = distance - CIRCLE_RADIUS
        k_heading_current = K_HEADING * (current_speed / SAFE_AIRSPEED)
        k_distance_current = K_DISTANCE * (current_speed / SAFE_AIRSPEED)

        # Wind correction (simplified)
        wind_effect = vehicle.airspeed - vehicle.groundspeed
        wind_correction = K_WIND * wind_effect

        # Centripetal force for circular path
        desired_bank_centripetal = degrees(atan2(current_speed**2, g * CIRCLE_RADIUS))

        # Combined bank (removed K_ROLL_CIRCLE term as it’s redundant with distance correction)
        desired_bank = -(k_heading_current * heading_error + k_distance_current * error_distance +
                         desired_bank_centripetal + wind_correction)
        desired_bank = max(min(desired_bank, MAX_ROLL), -MAX_ROLL)
        desired_bank = 0.7 * last_bank + 0.3 * desired_bank
        last_bank = desired_bank

        # Pitch control
        desired_descent_rate = current_speed / GLIDE_RATIO if current_speed > 0 else 0
        filtered_descent_rate = alpha * last_descent_rate + (1 - alpha) * actual_descent_rate
        last_descent_rate = filtered_descent_rate
        error_descent = desired_descent_rate - filtered_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)

        send_attitude_target(vehicle, desired_bank, desired_pitch, current_yaw)

        if time.time() - last_print_time > 0.5:
            logging.info(f"Circling - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Bank: {desired_bank:.1f}°, Speed: {current_speed:.1f}m/s")
            print(f"Circling - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Bank: {desired_bank:.1f}°, Speed: {current_speed:.1f}m/s")
            last_print_time = time.time()
        time.sleep(CONTROL_FREQ)

    if time.time() - circle_start_time >= MAX_CIRCLE_TIME:
        logging.warning("Circling timeout. Proceeding to final approach.")
        print("Circling timeout. Proceeding to final approach.")

    # Final approach
    logging.info("Circling complete. Approaching waypoint...")
    print("Circling complete. Approaching waypoint...")
    while True:
        set_throttle_off(vehicle)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_yaw = vehicle.heading
        vertical_speed = vehicle.velocity[2] if vehicle.velocity is not None else 0
        if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt]):
            print("Invalid sensor data. Skipping cycle.")
            time.sleep(CONTROL_FREQ)
            continue
        distance = get_distance_metres(current_loc, waypoint)

        if current_alt < 2.2 and abs(vertical_speed) < 0.1:
            final_distance = get_distance_metres(current_loc, waypoint)
            logging.info("Landing detected.")
            print("Landing detected.")
            vehicle.mode = VehicleMode("MANUAL")
            while vehicle.mode != "MANUAL":
                time.sleep(1)
            logging.info("Mode set to MANUAL. Ready to disarm...")
            print("Mode set to MANUAL. Ready to disarm...")
            vehicle.armed = False
            logging.info(f"Final distance from waypoint: {final_distance:.1f}m")
            print(f"Final distance from waypoint: {final_distance:.1f}m")
            break

        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)
        desired_pitch = base_pitch
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)

        if time.time() - last_print_time > 1:
            logging.info(f"Final - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Pitch: {desired_pitch:.1f}°")
            print(f"Final - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Pitch: {desired_pitch:.1f}°")
            last_print_time = time.time()
        time.sleep(CONTROL_FREQ)

    logging.info("Landing complete.")
    print("Landing complete.")
    vehicle.channels.overrides = {}

def main():
    parser = argparse.ArgumentParser(description='Glide to waypoint with throttle off, circling CCW.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    parser.add_argument('--sitl', action='store_true', help="Run in SITL mode")
    args = parser.parse_args()

    global HORIZONTAL_TOLERANCE, VERTICAL_TOLERANCE
    if args.sitl:
        HORIZONTAL_TOLERANCE = 100.0
        VERTICAL_TOLERANCE = 10.0

    MAX_RETRIES = 5
    vehicle = None
    for attempt in range(MAX_RETRIES):
        try:
            logging.info(f"Connecting to vehicle on {args.connect}...")
            print(f"Connecting to vehicle on {args.connect}...")
            vehicle = connect(args.connect, wait_ready=['location', 'airspeed'], timeout=30)
            break
        except Exception as e:
            logging.error(f"Connection attempt {attempt + 1} failed: {e}")
            print(f"Connection attempt {attempt + 1} failed: {e}")
            if attempt == MAX_RETRIES - 1:
                raise Exception("Failed to connect to vehicle")
            time.sleep(5)

    try:
        wait_for_manual_arm(vehicle)
        takeoff_check(vehicle)
        start_location = get_start_location_input()
        navigate_to_start_location(vehicle, start_location)
        waypoint = get_waypoint_input(vehicle)
        glide_to_waypoint(vehicle, waypoint)
    finally:
        if vehicle:
            vehicle.channels.overrides = {}
            vehicle.close()
            logging.info("Vehicle connection closed.")
            print("Vehicle connection closed.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("User interrupted. Cleaning up...")
        print("User interrupted. Cleaning up...")
    except Exception as e:
        logging.error(f"Error: {e}")
        print(f"Error: {e}")
