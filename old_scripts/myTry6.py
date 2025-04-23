from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import argparse
import logging
from math import sin, cos, sqrt, atan2, radians, degrees, pi

# Configuration Constants
GLIDE_RATIO = 10.0        # Glide ratio (distance/altitude)
CIRCLE_RADIUS = 50.0      # Desired circling radius (meters)
IDEAL_HEIGHT = 10.0       # Height to start final approach (meters)
K_ROLL = 1.7              # Roll gain
K_P = 0.1                 # Pitch gain
K_DISTANCE = 3.5          # Distance based control while circling
THROTTLE_MIN = 1000       # PWM for throttle off
CONTROL_FREQ = 0.05       # Control loop frequency (20 Hz)
MAX_ROLL = 55.0           # Max roll angle (degrees)
MAX_PITCH = 5.0           # Max pitch angle (degrees)
MIN_PITCH = -15.0         # Min pitch angle (degrees)
MIN_TAKEOFF_ALT = 5.0     # Min altitude for takeoff (meters)
MIN_START_ALT = 5.0       # Min starting altitude (meters)
HORIZONTAL_TOLERANCE = 50.0  # Horizontal tolerance (meters)
VERTICAL_TOLERANCE = 5.0    # Vertical tolerance (meters)
ALPHA = 0.8               # Descent rate filter coefficient
FLAIR_ALTITUDE = 5.0      # Altitude to start flair (meters)
K_DESCENT = 0.5           # Descent rate scaling factor (m/s per meter)

# Setup logging
logging.basicConfig(filename='flight_log.log', level=logging.INFO,
                    format='%(asctime)s %(message)s')

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
    """Send attitude commands via MAVLink."""
    q = quaternion_from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg))
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0b00000111, q, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)

def set_throttle_off(vehicle):
    """Set throttle to minimum."""
    vehicle.channels.overrides['3'] = THROTTLE_MIN

def get_distance_metres(location1, location2):
    """Calculate distance between two locations."""
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
    """Calculate base pitch for glide ratio."""
    return -degrees(atan2(1, glide_ratio))

def wait_for_manual_arm(vehicle):
    """Wait for manual arming."""
    logging.info("Waiting for manual arming...")
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    logging.info("Vehicle armed!")
    print("Vehicle armed!")

def takeoff_check(vehicle):
    """Wait for takeoff confirmation."""
    logging.info(f"Waiting for takeoff (alt > {MIN_TAKEOFF_ALT}m, speed > 5m/s)...")
    print(f"Waiting for takeoff (alt > {MIN_TAKEOFF_ALT}m, speed > 5m/s)...")
    while (vehicle.location.global_relative_frame.alt < MIN_TAKEOFF_ALT or
           vehicle.airspeed < 5.0):
        time.sleep(1)
    logging.info("Takeoff confirmed.")
    print("Takeoff confirmed.")

def get_start_location_input():
    """Get starting location from user with SW example."""
    while True:
        try:
            coord_input = input("Enter start location as 'lat,lon,alt' (e.g., SW: 35.726010,-78.697287,200): ")
            lat, lon, alt = map(float, coord_input.split(','))
            if not (-90 <= lat <= 90 and -180 <= lon <= 180) or alt < MIN_START_ALT:
                print("Invalid input. Try again.")
                continue
            return LocationGlobalRelative(lat, lon, alt)
        except ValueError:
            print("Invalid format. Use: lat,lon,alt")

def navigate_to_start_location(vehicle, start_location):
    """Navigate to start location with display data."""
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
            time.sleep(1)
            continue
        distance = get_distance_metres(current_loc, start_location)
        alt_diff = abs(current_alt - start_location.alt)
        if distance < HORIZONTAL_TOLERANCE and alt_diff < VERTICAL_TOLERANCE:
            logging.info("Start location reached.")
            print("Start location reached.")
            break
        logging.info(f"Dist to start: {distance:.1f}m, Alt diff: {alt_diff:.1f}m")
        print(f"Dist to start: {distance:.1f}m, Alt diff: {alt_diff:.1f}m")
        time.sleep(1)

def get_waypoint_input(vehicle):
    """Get landing waypoint from user with HOME example."""
    while True:
        try:
            coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., HOME: 35.727312,-78.696101): ")
            lat, lon = map(float, coord_input.split(','))
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                print("Invalid coordinates.")
                continue
            waypoint = LocationGlobal(lat, lon, 0)
            current_loc = vehicle.location.global_frame
            current_alt = vehicle.location.global_relative_frame.alt
            distance = get_distance_metres(current_loc, waypoint)
            required_alt = distance / GLIDE_RATIO
            if current_alt < required_alt:
                choice = input(f"Altitude too low ({current_alt:.1f}m < {required_alt:.1f}m). 1) Retry, 2) Proceed? ")
                if choice == '1':
                    continue
            return waypoint
        except ValueError:
            print("Invalid format. Use: lat,lon")

def glide_to_waypoint(vehicle, waypoint):
    """Guide drone to waypoint with throttle off, maintaining CCW circling."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    logging.info("Starting glide...")
    print("Starting glide...")
    
    base_pitch = calculate_glide_pitch(GLIDE_RATIO)
    last_print_time = time.time()
    last_descent_rate = 0.0
    
    # Glide towards waypoint
    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_yaw = vehicle.heading
        ground_speed = vehicle.groundspeed
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
        if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt, ground_speed]):
            time.sleep(CONTROL_FREQ)
            continue
        distance = get_distance_metres(current_loc, waypoint)
        set_throttle_off(vehicle)
    
        if distance <= CIRCLE_RADIUS:
            break
    
        # Glide phase: Head directly toward waypoint
        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)
    
        # Dynamic pitch control
        desired_descent_rate = ground_speed / GLIDE_RATIO if ground_speed > 0 else 0
        filtered_descent_rate = ALPHA * last_descent_rate + (1 - ALPHA) * actual_descent_rate
        last_descent_rate = filtered_descent_rate
        error_descent = desired_descent_rate - filtered_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)
    
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)
    
        if time.time() - last_print_time > 1:
            print(f"Glide - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Error: {heading_error:.1f}, Roll: {desired_roll:.1f}┬░, Pitch: {desired_pitch:.1f}┬░")
            last_print_time = time.time()
        time.sleep(CONTROL_FREQ)
    
    # CCW circling phase: Maintain CCW orbit until IDEAL_HEIGHT
    logging.info("Starting CCW circling...")
    print("Starting CCW circling...")
    while True:
        # Get current state
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_yaw = vehicle.heading
        ground_speed = vehicle.groundspeed
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
        if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt, ground_speed]):
            time.sleep(CONTROL_FREQ)
            continue
        distance = get_distance_metres(current_loc, waypoint)
        set_throttle_off(vehicle)
        
        # Exit condition
        if current_alt <= IDEAL_HEIGHT:
            break
        
        # Calculate errors
        e_d = distance - CIRCLE_RADIUS  # Distance error
        bearing = calculate_bearing(current_loc, waypoint)
        
        # Heading adjustment for distance correction
        heading_adjustment = -K_DISTANCE * e_d
        heading_adjustment = max(min(heading_adjustment, 30), -30)  # Clamp to [-30, 30] degrees
        
        # Desired heading with adjustment for CCW orbit
        desired_heading = (bearing + 90 + heading_adjustment) % 360
        
        # Angular error
        heading_error = (desired_heading - current_yaw + 180) % 360 - 180
        
        # Desired roll
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)
        
        # Dynamic pitch control (unchanged from glide phase)
        desired_descent_rate = ground_speed / GLIDE_RATIO if ground_speed > 0 else 0
        filtered_descent_rate = ALPHA * last_descent_rate + (1 - ALPHA) * actual_descent_rate
        last_descent_rate = filtered_descent_rate
        error_descent = desired_descent_rate - filtered_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)
        
        # Send control commands
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)
        
        # Debug output
        if time.time() - last_print_time > 1:
            print(f"Circling - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, heading_adj: {heading_adjustment:.1f}┬░, Roll: {desired_roll:.1f}┬░")
            last_print_time = time.time()
        time.sleep(CONTROL_FREQ)
    
    # Final approach with flair
    logging.info("Approaching waypoint...")
    print("Approaching waypoint...")
    while True:
        set_throttle_off(vehicle)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_yaw = vehicle.heading
        vertical_speed = vehicle.velocity[2] if vehicle.velocity is not None else 0
        if any(v is None for v in [current_loc.lat, current_loc.lon, current_alt]):
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
            vehicle.armed = False
            print(f"Final distance: {final_distance:.1f}m")
            break
        
        bearing = calculate_bearing(current_loc, waypoint)
        heading_error = (bearing - current_yaw + 180) % 360 - 180
        desired_roll = K_ROLL * heading_error
        desired_roll = max(min(desired_roll, MAX_ROLL), -MAX_ROLL)
    
        # Dynamic pitch control with flair
        ground_speed = vehicle.groundspeed
        actual_descent_rate = vehicle.velocity[2] if vehicle.velocity is not None else 0
        normal_descent_rate = ground_speed / GLIDE_RATIO if ground_speed > 0 else 0
        if current_alt < FLAIR_ALTITUDE:
            max_descent_rate = K_DESCENT * current_alt
            desired_descent_rate = min(normal_descent_rate, max_descent_rate)
        else:
            desired_descent_rate = normal_descent_rate
        filtered_descent_rate = ALPHA * last_descent_rate + (1 - ALPHA) * actual_descent_rate
        last_descent_rate = filtered_descent_rate
        error_descent = desired_descent_rate - filtered_descent_rate
        pitch_adjustment = K_P * error_descent
        desired_pitch = base_pitch + pitch_adjustment
        desired_pitch = max(min(desired_pitch, MAX_PITCH), MIN_PITCH)
    
        send_attitude_target(vehicle, desired_roll, desired_pitch, current_yaw)
    
        if time.time() - last_print_time > 1:
            print(f"Final Approach - Dist: {distance:.1f}m, Alt: {current_alt:.1f}m, Desired Descent Rate: {desired_descent_rate:.2f}m/s")
            last_print_time = time.time()
        time.sleep(CONTROL_FREQ)
    
    logging.info("Landing complete.")
    print("Landing complete.")
    vehicle.channels.overrides = {}

def main():
    parser = argparse.ArgumentParser(description='Glide to waypoint with CCW circling.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    parser.add_argument('--sitl', action='store_true', help="Run in SITL mode")
    args = parser.parse_args()

    vehicle = connect(args.connect, wait_ready=['location', 'airspeed'], timeout=30)
    try:
        wait_for_manual_arm(vehicle)
        takeoff_check(vehicle)
        start_location = get_start_location_input()
        navigate_to_start_location(vehicle, start_location)
        waypoint = get_waypoint_input(vehicle)
        glide_to_waypoint(vehicle, waypoint)
    finally:
        vehicle.close()
        print("Vehicle connection closed.")

if __name__ == "__main__":
    main()
