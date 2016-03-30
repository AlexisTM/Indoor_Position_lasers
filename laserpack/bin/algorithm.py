#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
algorithm.py

This script is the Python implementation of the position algorithm.

It converts 4 or 6 readings into positions and the yaw, filtering 
values via a Kalman Filter

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
import mavros
import tf
from transformations import *
from algorithm_functions import *
from laserpack.msg import distance
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from threading import Thread

# update IMU
def imu_callback(data):
    global imu
    imu = data

# Handle lasers
def raw_lasers_callback(data):
    global raw
    global imu
    global positions
    global orientations
    global laserNumber
    global pub_position
    # preCorrectionLasers is TODO
    raw = preCorrectionLasers(data)
    
    print "raw : ", raw

    # convert imu to a quaternion tuple
    quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    q = quaternion_from_euler(roll, pitch, 0, axes="sxyz")

    laser1x = quaternionRotation(positions[0], q)
    orientation1x = quaternionRotation(orientations[0], q)
    laser2x = quaternionRotation(positions[1], q)
    orientation2x = quaternionRotation(orientations[1], q)

    yawMeasured =  getYawInXL(laser1x, orientation1x, raw.lasers[0], orientation_lengths[0], laser2x, orientation2x, raw.lasers[1], orientation_lengths[1])

    q = quaternion_from_euler(roll, pitch, yawMeasured, axes="sxyz")

    # Rotate ALL vectors
    local_positions = list()
    local_orientations = list()
    local_targets = list()
    for i in range(laserNumber):
        local_positions.append(quaternionRotation(positions[i], q))
        local_orientations.append(quaternionRotation(orientations[i], q))
        local_targets.append(extrapolate(local_positions[i], local_orientations[i], raw.lasers[i]))


    # TODO Add an output filter (Kalman filter on positions and yaw)
    msg = PoseStamped()
    msg.pose.position.x = (local_targets[0][0] + local_targets[1][0])/2
    if(laserNumber == 4) :
        msg.pose.position.y = local_targets[2][1]
        msg.pose.position.z = local_targets[3][2]
    else : 
        msg.pose.position.y = (local_targets[2][1] + local_targets[3][1])/2
        msg.pose.position.z = (local_targets[3][2] + local_targets[4][2])/2
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    pub_position(msg)


def preCorrectionLasers(data):
    global raw
    global lasers_offsets
    global laserNumber
    # TODO Handle data, keep old data if outlier/readerror detected
    # TODO Then, edit raw with correct values
    # TODO Eventual resize data (/100)
    deoffset = list()
    for i in range(laserNumber):
        # Divide by 100 => centimeters to meters
        deoffset.append((data.lasers[i]*offset[i][0] + offset[i][1])/100)
    return deoffset

# init should get values of posiion of the lasers
def init():
    global raw
    global positions
    global orientations
    global laserNumber
    global orientation_lengths
    global lasers_offsets
    rospy.init_node('position_algorithm')
    raw = distance()
    laserNumber = 4
    positions = ((-15,-21,-4),       \
                 (7,20,-4),          \
                 (-31,-2,-4),        \
                 (32,-2,-4),         \
                 (-13,-21,-7.5),     \
                 (10,20,-7.5))
    orientations = ((1,0,0), (1,0,0), (0,1,0), (0,1,0), (0,0,1), (0,0,1))
    orientation_lengths = (length(orientations[0]), length(orientations[1]), length(orientations[2]), length(orientations[3]), length(orientations[4]), length(orientations[5]))
    lasers_offsets =   ((1,-4.45), \
                        (1,-1.489), \
                        (1,0), \
                        (1,0), \
                        (1,0), \
                        (1,0))
	

# listerners listen to ROS
def subscribers():
    global pub_position
    imu_sub         = rospy.Subscriber('mavros/imu/data', Imu, imu_callback)
    state_sub       = rospy.Subscriber('lasers/raw', distance, raw_lasers_callback)
    pub_position    = rospy.Publisher('lasers/pose', PoseStamped, queue_size=3)
	

def main():
    while not rospy.is_shutdown(): 
        raw_input("Press anything to quit")
        # rospy.spin()
        break

if __name__ == '__main__':
    rospy.loginfo("Position algorithm started")
    try:
        init()
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass

