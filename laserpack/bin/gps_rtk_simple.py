#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
gps_rtk_simple.py

This script show how we can get the RTK position from the Piksi and rotate it to
fits the same yaw. The yaw angle used in this project is perpendicular to walls.

This file is part of ILPS (Indoor Laser Positioning System).

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

Copyright (c) Alexis Paques 2016
"""

from __future__ import division
import rospy
import tf
from time import time
from transformations import *
from algorithm_functions import *
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from sensor_msgs import NavSatFix

# Laser position callack to extract the yaw
def laser_position_callback(data):
    global yaw_uav_rad
    quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    _, _, yaw_uav_rad = euler_from_quaternion(quaternion, axes="sxyz")

def gps_odometry_callback(data):
    global yaw_uav_rad
    odometry = data
    vector = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    axis = [0,0,1] # Around axis Z
    rotationAxisAngle(vector, axis, yaw_uav_rad):

def gps_precision_callback(data):
    precision = data

# init fonction
def init():
    # Input data
    # Output data
    global yaw_uav_rad, precision, odometry, rotated_odometry
    # Objects
    # Publishers

    precision = NavSatFix()
    odometry = Odometry()
    rotated_odometry = Odometry()
    # in radians
    yaw_uav_rad = 0

# listerners listen to ROS
def subscribers():
    # Input data
    # Output data
    # Subscribers/publishers
    global sub_filtered
    # Objects

    # Node initiation
    rospy.init_node('gps_acquisition_simple')

    sub_filtered        = rospy.Subscriber('lasers/filtered', PoseStamped, laser_position_callback)
    sub_gps_odometry    = rospy.Subscriber('gps/rtkfix', Odometry, gps_odometry_callback)
    sub_gps_satfix      = rospy.Subscriber('gps/fix', NavSatFix, gps_precision_callback)


def main():
    # Output data
    global yawprint
    while not rospy.is_shutdown():
        #rospy.spin()
        what = getch()
        if what == "q":
            break
        print "m:", yawprint[0]
        print "y:", yawprint[1]
        # rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Position algorithm started")
    try:
        init()
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
