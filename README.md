# ECE492-AutonomousSystems

Drone Glide and Circle Landing Controller
This program controls a drone to glide towards a specified waypoint, perform counter-clockwise circling around the waypoint until a certain height, and then land. It uses the DroneKit library to interface with the drone.
Prerequisites

Python 3.x
DroneKit library
MAVLink compatible drone or SITL (Software In The Loop) simulation

Installation

Install DroneKit:
pip install dronekit


If using SITL, install DroneKit-SITL:
pip install dronekit-sitl


Clone or download this project to your local machine.


Setup and Usage

Open QGroundControl for monitoring.
Navigate to the ardupilot directory.
Run sim_vehicle.py -L NCSU -v ArduPlane --out="udp:127.0.0.1:14552" to start the ArduPlane simulation.
Execute python3 AutoGliderProj.py --connect :14552 to connect the script to the simulation.

Alternatively, you can connect to a real drone by specifying the appropriate connection string with the --connect flag.
Configuration Parameters
The following constants can be adjusted in AutoGliderProj.py to modify the drone's behavior:

GLIDE_RATIO: The glide ratio of the drone (distance/altitude). Default: 10.0

CIRCLE_RADIUS: The desired radius for circling around the waypoint in meters. Default: 50.0

IDEAL_HEIGHT: The height at which to start the final approach in meters. Default: 10.0

K_ROLL: Gain for roll control. Default: 2.0

K_P: Gain for pitch control. Default: 0.1

K_DISTANCE: Gain for distance-based control during circling. Default: 3.5

THROTTLE_MIN: PWM value for throttle off. Default: 1000

CONTROL_FREQ: Frequency of the control loop in seconds. Default: 0.05 (20 Hz)

MAX_ROLL: Maximum roll angle in degrees. Default: 55.0

MAX_PITCH: Maximum pitch angle in degrees. Default: 5.0

MIN_PITCH: Minimum pitch angle in degrees. Default: -15.0

MIN_TAKEOFF_ALT: Minimum altitude for takeoff confirmation in meters. Default: 5.0

MIN_START_ALT: Minimum starting altitude in meters. Default: 5.0

HORIZONTAL_TOLERANCE: Horizontal tolerance for reaching the start location in meters. Default: 50.0

VERTICAL_TOLERANCE: Vertical tolerance for reaching the start location in meters. Default: 5.0

ALPHA: Coefficient for descent rate filter. Default: 0.8

Logging
The program logs flight data to 'flight_log.log' in the current directory. Each log entry includes a timestamp and a message describing the event or status.
Notes

Ensure the drone is properly calibrated and configured before running the script.
The program assumes the drone is capable of gliding with the throttle off.
The circling is counter-clockwise; adjust the code if clockwise circling is desired.
The final landing detection is based on altitude and vertical speed; adjust thresholds if necessary.


