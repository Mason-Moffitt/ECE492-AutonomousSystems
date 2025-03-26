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


class mission_item:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2
        self.param3 = 20
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

def arm(the_connection):
    print ("-- Arming")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")

def takeoff_mode(the_connection):
    print("-- Takeoff Mode")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, int(13), 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")

def takeoff(the_connection):
    print("-- Takeoff Initiated")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 10, 0, 0, math.nan, 0, 0, 10)

    ack(the_connection, "COMMAND_ACK")

def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending Message out")

    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n, 0)

    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("-- Creating a waypoint")

        the_connection.mav.mission_item_send(the_connection.target_system, the_connection.target_component,
                                             waypoint.seq,
                                             waypoint.frame,
                                             waypoint.command,
                                             waypoint.current,
                                             waypoint.auto,
                                             waypoint.param1,
                                             waypoint.param3,
                                             waypoint.param4,
                                             waypoint.param5,
                                             waypoint.param6,
                                             waypoint.param7,
                                             waypoint.mission_type)
        
    if waypoint != mission_items[n-1]:
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSON_ACK")

def set_return(the_connection):
    print("-- Set Return to Launch")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")

def start_mission(the_connection):
    print("-- Mission Start")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

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

        # mission_waypoints = []

        # mission_waypoints.append(mission_item(0, 0, 35.725603, -78.699720, 20))
        # mission_waypoints.append(mission_item(1, 0, 35.735603, -78.699720, 10))

        # upload_mission(the_connection, mission_waypoints)

        takeoff_mode(the_connection)

        arm(the_connection)

        # takeoff(the_connection)

        # start_mission(the_connection)

        # for mission_item in mission_waypoints:
        #     # print("-- Message Read " + str(the_connection.recv_match(type='MISSION_ITEM_REACHED', condition='MISSION_ITEM_REACHED.SEQ == {0}'.format)))
        #     print("-- Message Read MISSION_ITEM_REACHED {seq : " + str(the_connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True).seq) + "}")


        # set_return(the_connection)