#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
usage: python square_off.py --connect <*connection_string>
This script connects to the drone and waits until armed. When armed it will takeoff
to 3m altitude, then navigate a 10x10 meter square. At each corner of the square the drone
will wait for 5 seconds.
"""

from __future__ import print_function

import math
# import time
# import sys
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect


def arm(the_connection):
    print ("-- Arming")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")


def takeoff(the_connection):
    print("-- Takeoff Initiated")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 10, 0, 0, math.nan, 0, 0, 10)

    ack(the_connection, "COMMAND_ACK")


# def goto(the_connection):
#     print("-- Navigation Initiated")

#     # the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
#     #                                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 10, math.nan, 35.725603, -78.699720, 20)

#     the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
#                                          mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, -1, 1, 10, math.nan, 35.725603, -78.699720, 20)

#     # ack(the_connection, "")


def set_return(the_connection):
    print("-- Set Return to Launch")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")

def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))

if __name__ == "__main__":
    print("-- Program Started")
    the_connection = mavutil.mavlink_connection('udpin:localhost:14552')

    while(the_connection.target_system == 0):
        print("-- Checking Heartbeat")
        the_connection.wait_heartbeat()
        print("-- Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


        # create target location message
        message = dialect.MAVLink_mission_item_int_message(
            target_system=the_connection.target_system,
            target_component=the_connection.target_component,
            seq=0,
            frame=5,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=2,
            autocontinue=0,
            param1=0,
            param2=0,
            param3=2,
            param4=math.nan,
            x=int(35.725603 * 1e7),
            y=int(-78.699720 * 1e7),
            z=20
        )

        # send target location command to the vehicle
        the_connection.mav.send(message)
        


        # mission_waypoints = []

        # mission_waypoints.append(mission_item(0, 0, 35.725603, -78.699720, 20))
        # mission_waypoints.append(mission_item(1, 0, 35.735603, -78.699720, 10))

        # arm(the_connection)

        # takeoff(the_connection)

        # goto(the_connection)

        # set_return(the_connection)