from dronekit import connect, VehicleMode, LocationGlobal
import time
import argparse
from math import sin, cos, sqrt, atan2, radians

# Constants (adjustable)
GLIDE_RATIO = 10.0        # Horizontal distance per unit altitude loss
IDEAL_RADIUS = 100.0      # Distance from waypoint to start circling (meters)
IDEAL_HEIGHT = 10.0       # Target height to stop circling (meters)
IDEAL_SPEED = 10.0        # Target speed to stop circling (m/s)
CIRCLE_RADIUS = 50.0      # Radius for circling pattern (meters)

def get_distance_metres(location1, location2):
    """Calculate horizontal distance between two LocationGlobal points in meters."""
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

def wait_for_manual_arm(vehicle):
    """Wait for the vehicle to be manually armed."""
    print("Waiting for manual arming...")
    while not vehicle.armed:
        time.sleep(1)
    print("Vehicle is armed!")

def takeoff_check(vehicle):
    """Perform takeoff and confirm airborne status."""
    print("Setting mode to TAKEOFF...")
    vehicle.mode = VehicleMode("TAKEOFF")
    while vehicle.mode != "TAKEOFF":
        time.sleep(1)
    print("Waiting for plane to accelerate and climb...")
    # Wait until altitude > 5m and airspeed > 5m/s to confirm takeoff
    while vehicle.location.global_relative_frame.alt < 5.0 or vehicle.airspeed < 5.0:
        time.sleep(1)
    print("Takeoff confirmed.")

def get_waypoint_input(vehicle):
    """Get and validate waypoint from user, ensuring it's within gliding reach."""
    while True:
        coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., 35.7721,-78.6745): ")
        try:
            lat, lon = map(float, coord_input.split(','))
            waypoint = LocationGlobal(lat, lon, 0)
            current_loc = vehicle.location.global_frame
            current_alt = vehicle.location.global_relative_frame.alt
            distance = get_distance_metres(current_loc, waypoint)
            max_glide_distance = GLIDE_RATIO * current_alt
            if distance > max_glide_distance:
                print(f"Waypoint {distance:.1f}m away exceeds glide range {max_glide_distance:.1f}m. Try again.")
                continue
            print(f"Waypoint accepted: {distance:.1f}m, within glide range {max_glide_distance:.1f}m.")
            return waypoint
        except ValueError:
            print("Invalid format. Use: lat,lon")
            continue

def set_throttle_off(vehicle):
    """Override throttle to zero (assuming channel 3 is throttle)."""
    vehicle.channels.overrides['3'] = 1000
    print("Throttle turned off.")

def calculate_circle_point(center, current_loc, radius, angle_deg):
    """Calculate a point on a circle around the center at given angle."""
    angle_rad = radians(angle_deg)
    dlat = (radius * cos(angle_rad)) / 111320  # Approx meters per degree lat
    dlon = (radius * sin(angle_rad)) / (111320 * cos(radians(center.lat)))  # Adjust for longitude
    return LocationGlobal(center.lat + dlat, center.lon + dlon, 0)

def glide_to_waypoint(vehicle, waypoint):
    """Manage the glide sequence including circling if needed."""
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    
    set_throttle_off(vehicle)
    print("Gliding towards waypoint...")

    while True:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        distance = get_distance_metres(current_loc, waypoint)

        # Step d: Glide until ideal radius hit
        if distance <= IDEAL_RADIUS:
            print(f"Reached ideal radius {IDEAL_RADIUS}m from waypoint.")
            break
        vehicle.simple_goto(waypoint)
        time.sleep(1)

    # Step e: Circle waypoint until ideal height and speed reached
    print("Circling waypoint to adjust height and speed...")
    angle = 0
    while current_alt > IDEAL_HEIGHT or current_speed > IDEAL_SPEED:
        circle_point = calculate_circle_point(waypoint, current_loc, CIRCLE_RADIUS, angle)
        vehicle.simple_goto(circle_point)
        time.sleep(1)
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        current_speed = vehicle.airspeed
        angle = (angle + 45) % 360  # Move 45 degrees per step
        print(f"Alt: {current_alt:.1f}m, Speed: {current_speed:.1f}m/s")

    # Step f: Stop circling, approach waypoint directly
    print("Stopping circle, approaching waypoint for landing...")
    vehicle.simple_goto(waypoint)

    # Step g: Glide down and land
    while current_alt > 1.0 and distance > 10.0:
        current_loc = vehicle.location.global_frame
        current_alt = vehicle.location.global_relative_frame.alt
        distance = get_distance_metres(current_loc, waypoint)
        time.sleep(1)
        print(f"Descending: Alt {current_alt:.1f}m, Distance {distance:.1f}m")

    print("Landing complete.")
    vehicle.channels.overrides.clear()  # Clear throttle override

def main():
    parser = argparse.ArgumentParser(description='Takeoff and glide-land at a specified waypoint.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=['location', 'airspeed'])

    # a. Wait for arming check
    wait_for_manual_arm(vehicle)

    # b. Takeoff check
    takeoff_check(vehicle)

    # c. Wait for destination waypoint and d. Ensure within reach
    waypoint = get_waypoint_input(vehicle)

    # e-g. Glide sequence
    glide_to_waypoint(vehicle, waypoint)

    vehicle.close()

if __name__ == "__main__":
    main()
