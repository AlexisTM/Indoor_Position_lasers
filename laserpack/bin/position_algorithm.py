#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
position_algorithm.py

DEPRECATRED : Used to control lasers, old way

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
 
import rospy
import mavros
import time
import tf
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from laserpack.msg import distance
from mavros.utils import *
from threading import Thread

# Linearity, offset
# The \ permit the line continuation

def publish_data():
    global pub_position
    global poseXYZ
    global imu
    msg = PoseStamped()
    msg.pose.position.x = poseXYZ[0]
    msg.pose.position.y = poseXYZ[1]
    msg.pose.position.z = poseXYZ[2]
    msg.pose.orientation.x = imu.orientation.x
    msg.pose.orientation.y = imu.orientation.y
    msg.pose.orientation.z = imu.orientation.z
    msg.pose.orientation.w = imu.orientation.w
    pub_position.publish(msg)
        #rate.sleep()

def setOffsets():
    global offset
    offset = [[1,-4.45], \
            [1,-1.489]]


def IMU_Callback(data):
    global imu
    imu = data

def floatify(data):
    floatified = []
    for laser in data :
        floatyone = float(laser)
        floatified.append(floatyone)
    return floatified

def offsetCorrection(data):
    global offset
    deoffset = []
    for laser in enumerate(data) :
        correct = (laser[1]*offset[laser[0]][0] + offset[laser[0]][1])
        deoffset.append(correct)
    return deoffset


def resizeData(data):
    resized = []
    for laser in data :
        correct = float(laser)/100
        resized.append(correct)
    return resized

def position(data):
    global imu
    XYZ = [0,0,0]
    mean = (data[0] + data[1])/2
    Q = (
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z,
        imu.orientation.w)
    euler = tf.transformations.euler_from_quaternion(Q)
    XYZ[2] = mean
    return XYZ

def LaserDistances_Callback(data):
    global statuses
    global poseXYZ
    statuses = data.status
    distances = data.lasers
    floatified = floatify(distances)
    deoffset_distances = offsetCorrection(floatified)
    resized_distances = resizeData(deoffset_distances)
    poseXYZ = position(resized_distances)
    publish_data()

def init():
    global poseXYZ
    global pub_position
    poseXYZ = []
    setOffsets()
    rospy.init_node('laserpack_position_algorithm')
    imu       = rospy.Subscriber('mavros/imu/data', Imu, IMU_Callback)
    time.sleep(1)
    raw_position       = rospy.Subscriber('lasers/raw', distance, LaserDistances_Callback)
    pub_position = rospy.Publisher('lasers/pose', PoseStamped,queue_size=3)
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Position algorithm Module starting")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.logerr("Position algorithm Module failed to start")
        pass