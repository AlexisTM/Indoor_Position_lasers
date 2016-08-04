#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
new_alogorithm_rtk.py

This script compute the multicopter position & send it via Mavros

ILPS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ILPS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with ILPS.  If not, see <http://www.gnu.org/licenses/>.

Software created by Alexis Paques and Nabil Nehri for the UCL
in a Drone-Based Additive Manufacturing of Architectural Structures
project financed by the MIT Seed Fund

Mavros by Vladimir Ermakov (c) 2015 under GNU GPLv3
Copyright (c) Alexis Paques 2016
"""

# Libraries
from __future__ import division
import rospy
import mavros
import time
from getch import *
from threading import Thread

# Messages
from geometry_msgs.msg import PoseStamped, Point
from laserpack.msg import Distance
from nav_msgs.msg import Odometry

def lasers_raw_callback(lasers_raw):
    global laser_altitude
    laser_altitude = float(lasers_raw.lasers[0] + lasers_raw.lasers[1]) / 200.0

# Piksi callback
def piksi_callback(odometry):
    global position_gps
    position_gps = data.pose.pose.position
    pass

# Thread sending the mocap position
def thread_mocap():
    global laser_altitude, position_gps, mocap_sent
    mocap_sent = 0
    rate = rospy.Rate(10)

    mocap_publisher = rospy.Publisher(
        'mavros/mocap/pose', PoseStamped, queue_size=1)

    while run:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = mocap_sent
        msg.pose.position.x = float(position_gps.x)
        msg.pose.position.y = float(position_gps.y)
        msg.pose.position.z = float(laser_altitude)
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        mocap_publisher.publish(msg)
        mocap_sent += 1
        rate.sleep()

def InterfaceKeyboard():
    global run
    what = getch()
    if what == "m":
        run = False
        time.sleep(1)
        exit()

def init():
    # Global initialisation
    global run, laser_altitude, position_gps
    run = True
    laser_altitude = 0
    position_gps = Point()

    # Subscribers
    laser_sub = rospy.Subscriber('lasers/raw', Distance, lasers_raw_callback)
    gps_sub = rospy.Subscriber('gps/rtkfix', Odometry, piksi_callback)

    # Threads
    tMocap = Thread(target=thread_mocap).start()

    # Minimal interface
    while not rospy.is_shutdown():
        InterfaceKeyboard()

if __name__ == '__main__':
    rospy.loginfo("Position algorithm")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
