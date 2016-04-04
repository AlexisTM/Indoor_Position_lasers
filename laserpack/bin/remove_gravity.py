#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
remove gravity.py

This script tries to remove gravity

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
Copyright (c) Nabil Nehri 2016
"""

from __future__ import division
import rospy
import mavros
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from getch import getch

# update IMU
def imu_callback(data):
    global timestampOld
    global linearAcceleration
    global gravity
    global linearAcceleration_imu
    timeConstant = 0.18
    alpha = 0.9
    timestamp = data.header.stamp.nsecs
    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z

    linearAcceleration_imu = [ax, ay, az]

    dt = (timestamp - timestampOld)/1000000000.0
    alpha = timeConstant / (timeConstant + dt)
    gravity[0] = alpha * gravity[0] + (1 - alpha) * ax
    gravity[1] = alpha * gravity[1] + (1 - alpha) * ay
    gravity[2] = alpha * gravity[2] + (1 - alpha) * az
 
    linearAcceleration[0] = ax - gravity[0]
    linearAcceleration[1] = ay - gravity[1]
    linearAcceleration[2] = az - gravity[2]
    TimestampOld = Timestamp





# init should get values of posiion of the lasers
def init():
    global timestampOld
    global linearAcceleration
    global gravity
    global linearAcceleration_imu
    rospy.init_node('Accel_Filtered')
    timestampOld = 0
    linearAcceleration =[0,0,0]
    gravity = [0,0,9.81]
    linearAcceleration_NonFiltered = [0,0,0]


# listerners listen to ROS
def subscribers():
    imu_sub         = rospy.Subscriber('mavros/imu/data', Imu, imu_callback)
    

def main():
    global linearAcceleration
    global linearAcceleration_imu
    while not rospy.is_shutdown(): 
        what = getch()
        if what == "q":
            break
        print "Ax_filtre:", linearAcceleration[0]
        print "Ay_filtre:", linearAcceleration[1]
        print "Az_filtre",  linearAcceleration[2]
        print "Ax_imu:", linearAcceleration_imu[0]
        print "Ay_imu:", linearAcceleration_imu[1]
        print "Az_imu",  linearAcceleration_imu[2]
        print "gravity_0:", gravity[0]
        print "gravity_1:", gravity[1]
        print "gravity_2",  gravity[2]
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