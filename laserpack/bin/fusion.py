#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
fusion.py

This script takes all the data all data the multicopter have to get the most
precise output position possible, taking in account the delays.

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
    # Objects

    # Node initiation
    rospy.init_node('sensor_fusion')


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
    rospy.loginfo("Sensor_fusion algorithm started")
    try:
        init()
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
