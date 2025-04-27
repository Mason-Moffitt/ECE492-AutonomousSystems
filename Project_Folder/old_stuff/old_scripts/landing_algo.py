from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
import math

# Script that flies up and lands, the throttle doesn't quite cooperate

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

def set_loiter_radius(vehicle, radius):
    print(f"Setting loiter radius to {radius} meters...")
    vehicle.parameters['WP_LOITER_RAD'] = radius
    while abs(vehicle.parameters['WP_LOITER_RAD'] - radius) > 0.1:
        print(" Waiting for parameter update...")
        time.sleep(1)
    print(f"Loiter radius set to {vehicle.parameters['WP_LOITER_RAD']} meters.")

def fly_to_coordinate(vehicle, lat, lon, alt):
    print(f"Flying to target: lat={lat}, lon={lon}, alt={alt}")
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location, airspeed=10)
    while True:
        distance = get_distance_metres(vehicle.location.global_relative_frame, location)
        if distance < 20:  # Within 20 meters of target
            break
        time.sleep(1)
    print("Reached target waypoint.")

def get_distance_metres(location1, location2):
    """Calculate distance between two LocationGlobal points in meters."""
    dlat = location2.lat - location1.lat
    dlon = location2.lon - location1.lon
    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5

def send_velocity(vehicle, north, east, down, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Velocity only
        0, 0, 0,
        north, east, down,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    time.sleep(duration)

def circle_and_descend(vehicle, radius=8, initial_altitude=50, descent_rate=1, landing_elevation=0):
    print(f"Starting circling descent with radius {radius}m from {initial_altitude}m")
    print(f"DEBUG: Initial parameters - radius={radius}, initial_altitude={initial_altitude}, descent_rate={descent_rate}, landing_elevation={landing_elevation}")
    
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)
    print("Vehicle in GUIDED mode.")
    
    # Adjust TECS parameters to allow descent
    try:
        vehicle.parameters['TECS_SINK_MAX'] = 5  # Allow up to 5 m/s descent
        vehicle.parameters['TECS_CLMB_MAX'] = 0  # Prevent climbing
        vehicle.parameters['THR_MIN'] = 0        # Ensure throttle can go to 0%
        vehicle.parameters['THR_SUPPRESS'] = 1   # Suppress throttle (Ardupilot-specific)
        print(f"DEBUG: TECS_SINK_MAX set to {vehicle.parameters['TECS_SINK_MAX']}")
        print(f"DEBUG: TECS_CLMB_MAX set to {vehicle.parameters['TECS_CLMB_MAX']}")
        print(f"DEBUG: THR_MIN set to {vehicle.parameters['THR_MIN']}")
        print(f"DEBUG: THR_SUPPRESS set to {vehicle.parameters['THR_SUPPRESS']}")
    except Exception as e:
        print(f"DEBUG: Failed to set parameters: {e}")
    
    # Set throttle to 0 (off)
    throttle_channel = '3'
    vehicle.channels.overrides[throttle_channel] = 1000  # Throttle off
    print("DEBUG: Throttle cut to 1000 PWM (off)")
    
    # Get center coordinates for circling
    center_lat = vehicle.location.global_relative_frame.lat
    center_lon = vehicle.location.global_relative_frame.lon
    current_altitude = initial_altitude
    
    airspeed = 10  # m/s
    circumference = 2 * math.pi * radius
    period = circumference / airspeed  # Time to complete one circle
    
    print(f"DEBUG: Center coordinates - lat={center_lat}, lon={center_lon}")
    print(f"DEBUG: Initial current_altitude set to {current_altitude}m")
    print(f"DEBUG: airspeed={airspeed}m/s, circumference={circumference:.1f}m, period={period:.1f}s")
    
    start_time = time.time()
    
    # Circle and descend until near ground
    while vehicle.location.global_relative_frame.alt > (landing_elevation + 0.5):  # Adjusted to 0.5m for ground contact
        elapsed_time = time.time() - start_time
        angle = (2 * math.pi * elapsed_time) / period  # Angle for circular path
        
        # Velocity commands for circling
        north_velocity = airspeed * math.cos(angle)
        east_velocity = airspeed * math.sin(angle)
        down_velocity = -descent_rate  # Negative for descent
        
        # Calculate target altitude (gradual descent)
        current_altitude = max(initial_altitude - (elapsed_time * descent_rate), landing_elevation)
        
        # Position command to reinforce circling and altitude
        lat_offset = (radius * math.cos(angle)) / 111111  # Convert meters to degrees
        lon_offset = (radius * math.sin(angle)) / (111111 * math.cos(math.radians(center_lat)))
        target_lat = center_lat + lat_offset
        target_lon = center_lon + lon_offset
        vehicle.simple_goto(LocationGlobalRelative(target_lat, target_lon, current_altitude), airspeed=airspeed)
        
        # Send velocity commands to maintain circling
        for _ in range(5):
            send_velocity(vehicle, north_velocity, east_velocity, down_velocity, 0.2)
        
        # Reapply throttle override to ensure it persists
        vehicle.channels.overrides[throttle_channel] = 1000  # Reapply in each loop
        
        actual_altitude = vehicle.location.global_relative_frame.alt
        pitch = vehicle.attitude.pitch * 180 / math.pi
        print(f"DEBUG: Loop - Target alt={current_altitude:.1f}m, Actual alt={actual_altitude:.1f}m, Down vel={down_velocity:.1f}m/s, Throttle=1000, Pitch={pitch:.1f}deg, Elapsed={elapsed_time:.1f}s, Mode={vehicle.mode.name}")
        time.sleep(0.1)  # Reduced sleep time to 0.1s for higher frequency
    
    vehicle.channels.overrides.clear()
    print("DEBUG: Altitude near ground. Descent complete, overrides cleared.")

def main():
    parser = argparse.ArgumentParser(description='Takeoff, fly to coordinate, circle and descend.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    coord_input = input("Enter target coordinates as 'lat,lon,alt' (e.g., 35.7721,-78.6745,100): ")
    try:
        lat, lon, alt = map(float, coord_input.split(','))
        if alt <= 5:
            print("Error: Altitude must be greater than 5m for descent.")
            return
    except ValueError:
        print("Invalid format. Use: lat,lon,alt")
        return

    landing_elevation = float(input("Enter landing site elevation ASL (meters, default 0): ") or 0)

    print(f"Connecting to vehicle on {args.connect}...")
    vehicle = connect(args.connect, wait_ready=True)

    wait_for_manual_arm(vehicle)
    set_takeoff_mode(vehicle)

    print("Waiting for plane to build speed and altitude...")
    time.sleep(10)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != "GUIDED":
        time.sleep(1)

    set_loiter_radius(vehicle, 8)
    fly_to_coordinate(vehicle, lat, lon, alt)

    circle_and_descend(vehicle, radius=8, initial_altitude=alt, descent_rate=1.5, landing_elevation=landing_elevation)

    print("Landing sequence complete. Monitor via QGroundControl.")
    time.sleep(10)

    print("Closing connection.")
    vehicle.close()

if __name__ == "__main__":
    main()