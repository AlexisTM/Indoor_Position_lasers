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
from transformations import euler_from_quaternion, quaternion_from_euler
from math import cos

# Messages
from geometry_msgs.msg import PoseStamped, Point
from laserpack.msg import Distance, RPY
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Imu callback
# Converts the quaternion to eulers for less compute power needed
def imu_callback(imu):
    global imu_euler, imu_quaternion
    imu_quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(imu_quaternion, axes="sxyz")
    imu_euler = RPY()
    imu_euler.roll = roll
    imu_euler.pitch = pitch
    imu_euler.yaw = yaw

# Lasers callback
# Basic pitch & roll correction
# sqrt(measure*cos(pitch)*measure*cos(pitch) + measure*cos(roll)*measure*cos(roll))
def lasers_raw_callback_altitude_only(lasers_raw):
    global laser_altitude
    mean_laser = float(lasers_raw.lasers[0] + lasers_raw.lasers[1]) / 200.0
    laser_altitude = ((mean_laser*cos(imu_euler.roll))**2 + (mean_laser*cos(imu_euler.pitch))**2)**0.5

# Piksi callback
def piksi_callback(odometry):
    global position_gps
    position_gps = odometry.pose.pose.position

# Thread sending the mocap position
def thread_mocap():
    global laser_altitude, position_gps, mocap_sent, imu_quaternion
    mocap_sent = 0
    rate = rospy.Rate(30)

    mocap_publisher = rospy.Publisher(
        'mavros/mocap/pose', PoseStamped, queue_size=1)

    while (run and not rospy.is_shutdown()):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = mocap_sent
        msg.pose.position.x = float(position_gps.x)
        msg.pose.position.y = float(position_gps.y)
        msg.pose.position.z = float(laser_altitude)
        msg.pose.orientation.x = imu_quaternion[0]
        msg.pose.orientation.y = imu_quaternion[1]
        msg.pose.orientation.z = imu_quaternion[2]
        msg.pose.orientation.w = imu_quaternion[3]
        mocap_publisher.publish(msg)
        mocap_sent += 1
        rate.sleep()

# Debug only sendsetpoints
def sendSetpoint():
    global run

    setPointsCount = 0
    local_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

    rate = rospy.Rate(20)

    while run:
        q = quaternion_from_euler(0, 0, 0, axes="sxyz")
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq=setPointsCount
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        local_setpoint_pub.publish(msg)
        setPointsCount = setPointsCount + 1
        rate.sleep()


# Minimale interface
def InterfaceKeyboard():
    global run
    what = getch()
    if what == "m":
        run = False
        time.sleep(1)
        exit()

# Initialisation
def init():
    # Global initialisation
    global run, laser_altitude, position_gps, imu_euler, imu_quaternion
    run = True
    laser_altitude = 0
    imu_quaternion = (0,0,0,1)
    position_gps = Point()
    imu_euler = RPY()

    rospy.init_node('algorithm_mocap_rtk')

    # Subscribers
    laser_sub   = rospy.Subscriber('lasers/raw', Distance, lasers_raw_callback_altitude_only)
    gps_sub     = rospy.Subscriber('gps/rtkfix', Odometry, piksi_callback)
    imu_sub     = rospy.Subscriber('mavros/imu/data', Imu, imu_callback)

    # Threads
    tMocap = Thread(target=thread_mocap).start()
    #tSetPoints = Thread(target=sendSetpoint).start()
    

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
