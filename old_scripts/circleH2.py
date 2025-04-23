from pymavlink import mavutil
from pymavlink.mavwp import MAVWPLoader
from pymavlink.mavutil import mavlink
import time
import argparse
import math
import threading
import os

# Constants for glide calculations
GLIDE_RATIO = 15.0  # 15:1 glide ratio (meters forward per meter descent)
OPTIMAL_GLIDE_AIRSPEED = 20.0  # m/s
SINK_RATE = OPTIMAL_GLIDE_AIRSPEED / GLIDE_RATIO  # m/s
CIRCLING_RADIUS = 200.0  # meters
MIN_SAFE_ALTITUDE = 50.0  # meters AGL
LANDING_ALTITUDE = 0.0  # meters AGL

# ArduPilot mode mapping (custom_mode to name)
ARDUPILOT_MODES = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    16: "INITIALISING",
    17: "LAND",
    18: "THERMAL",
    19: "QSTABILIZE",
    20: "QHOVER",
    21: "QLOITER",
    22: "QRTL",
    23: "QAUTOTUNE",
    24: "QACRO"
}

# Global variables for telemetry
telemetry_running = True
current_airspeed = 0.0
current_altitude_agl = 0.0
distance_to_waypoint = 0.0

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two lat/lon points in meters."""
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def get_vehicle_mode(vehicle):
    """Get the current vehicle mode from HEARTBEAT message."""
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg is None:
        print("Warning: No HEARTBEAT received for mode check.")
        return None
    custom_mode = msg.custom_mode
    return ARDUPILOT_MODES.get(custom_mode, "UNKNOWN")

def wait_for_manual_arm(vehicle):
    """Wait for the vehicle to be manually armed."""
    print("Waiting for manual arming...")
    while True:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg is None:
            print("No HEARTBEAT received. Check connection.")
            continue
        is_armed = (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        if is_armed:
            print("Vehicle is armed!")
            break
        time.sleep(1)

def display_telemetry(vehicle, target_lat, target_lon):
    """Run in a separate thread to display telemetry data."""
    global current_airspeed, current_altitude_agl, distance_to_waypoint, telemetry_running
    while telemetry_running:
        # Get airspeed from VFR_HUD
        msg_vfr = vehicle.recv_match(type='VFR_HUD', blocking=True, timeout=1)
        if msg_vfr:
            current_airspeed = msg_vfr.airspeed
        else:
            current_airspeed = 0.0

        # Get altitude AGL and position from GLOBAL_POSITION_INT
        msg_pos = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg_pos:
            current_altitude_agl = msg_pos.relative_alt / 1000.0  # mm to meters
            current_lat = msg_pos.lat / 1e7
            current_lon = msg_pos.lon / 1e7
            distance_to_waypoint = haversine_distance(current_lat, current_lon, target_lat, target_lon)
        else:
            current_altitude_agl = 0.0
            distance_to_waypoint = 0.0

        # Clear terminal and display telemetry
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"Telemetry:")
        print(f"Airspeed: {current_airspeed:.2f} m/s")
        print(f"Altitude AGL: {current_altitude_agl:.2f} m")
        print(f"Distance to Waypoint: {distance_to_waypoint:.2f} m")
        time.sleep(0.5)

def set_throttle(vehicle, throttle):
    """Set throttle value (0 to 1)."""
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        3,  # Throttle channel
        throttle * 1000 + 1000,  # PWM 1000-2000
        0, 0, 0, 0, 0
    )

def can_glide_to_target(altitude_agl, distance, airspeed):
    """Check if the aircraft can glide to the target."""
    required_altitude = distance / GLIDE_RATIO + LANDING_ALTITUDE
    altitude_ok = altitude_agl >= required_altitude
    airspeed_ok = 0.9 * OPTIMAL_GLIDE_AIRSPEED <= airspeed <= 1.1 * OPTIMAL_GLIDE_AIRSPEED
    return altitude_ok and airspeed_ok

def navigate_to_circle(vehicle, target_lat, target_lon, radius, altitude):
    """Command the vehicle to loiter around the waypoint."""
    vehicle.mav.set_position_target_global_int_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b110111111000,
        int(target_lat * 1e7),
        int(target_lon * 1e7),
        altitude,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        0,
        0, radius, 0, 0,
        target_lat, target_lon, altitude
    )

def glide_to_landing(vehicle, target_lat, target_lon, altitude):
    """Command the vehicle to glide to the waypoint and land."""
    vehicle.mav.set_position_target_global_int_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b110111111000,
        int(target_lat * 1e7),
        int(target_lon * 1e7),
        LANDING_ALTITUDE,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            distance = haversine_distance(current_lat, current_lon, target_lat, target_lon)
            if distance < 50:
                vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    17,  # LAND mode
                    0, 0, 0, 0, 0
                )
                break
        time.sleep(1)

def main():
    parser = argparse.ArgumentParser(description='Takeoff and glide-land at a specified waypoint.')
    parser.add_argument('--connect', default='127.0.0.1:14552', help="Connection string")
    args = parser.parse_args()

    # Handle connection string
    connect_str = args.connect
    if connect_str.startswith(':'):
        connect_str = '127.0.0.1' + connect_str
    print(f"Connecting to vehicle on {connect_str}...")
    try:
        vehicle = mavutil.mavlink_connection(connect_str)
        vehicle.wait_heartbeat(timeout=10)
        print("Vehicle connected!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    try:
        wait_for_manual_arm(vehicle)

        print("Setting mode to TAKEOFF...")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            13,  # TAKEOFF mode
            0, 0, 0, 0, 0
        )
        timeout = time.time() + 10
        while get_vehicle_mode(vehicle) != "TAKEOFF" and time.time() < timeout:
            time.sleep(1)
        if get_vehicle_mode(vehicle) != "TAKEOFF":
            print("Failed to set TAKEOFF mode.")
            vehicle.close()
            return

        print("Waiting for plane to accelerate and climb...")
        time.sleep(10)

        print("Switching to GUIDED mode...")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            15,  # GUIDED mode
            0, 0, 0, 0, 0
        )
        timeout = time.time() + 10
        while get_vehicle_mode(vehicle) != "GUIDED" and time.time() < timeout:
            time.sleep(1)
        if get_vehicle_mode(vehicle) != "GUIDED":
            print("Failed to set GUIDED mode.")
            vehicle.close()
            return

        # Ask for landing target
        coord_input = input("Enter landing coordinates as 'lat,lon' (e.g., 35.7721,-78.6745): ")
        try:
            target_lat, target_lon = map(float, coord_input.split(','))
        except ValueError:
            print("Invalid format. Use: lat,lon")
            vehicle.close()
            return

        # Set throttle to 0
        set_throttle(vehicle, 0.0)
        print("Throttle set to 0. Navigating with glide.")

        # Start telemetry display
        telemetry_thread = threading.Thread(target=display_telemetry, args=(vehicle, target_lat, target_lon))
        telemetry_thread.daemon = True
        telemetry_thread.start()

        # Get initial position and altitude
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            print("Failed to get position.")
            vehicle.close()
            return
        current_altitude_agl = msg.relative_alt / 1000.0

        # Navigate to circling path
        if current_altitude_agl < MIN_SAFE_ALTITUDE:
            print(f"Altitude too low ({current_altitude_agl:.2f}m). Must be above {MIN_SAFE_ALTITUDE}m.")
            vehicle.close()
            return

        print(f"Circling waypoint at {CIRCLING_RADIUS}m radius, altitude {current_altitude_agl:.2f}m...")
        navigate_to_circle(vehicle, target_lat, target_lon, CIRCLING_RADIUS, current_altitude_agl)

        # Monitor for glide conditions
        while True:
            if can_glide_to_target(current_altitude_agl, CIRCLING_RADIUS, current_airspeed):
                print("Optimal glide conditions met. Initiating glide to landing...")
                break
            msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                current_altitude_agl = msg.relative_alt / 1000.0
            time.sleep(1)

        # Stop telemetry
        telemetry_running = False
        telemetry_thread.join()

        # Glide to landing
        glide_to_landing(vehicle, target_lat, target_lon, LANDING_ALTITUDE)

        # Wait for landing
        print("Waiting for landing...")
        timeout = time.time() + 60
        while time.time() < timeout:
            msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg and (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == 0:
                print("Landing complete. Vehicle disarmed.")
                break
            time.sleep(1)

    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        vehicle.close()

if __name__ == "__main__":
    main()
