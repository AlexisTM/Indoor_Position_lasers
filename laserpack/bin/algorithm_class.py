#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
algorithm_class.py

This script is the Python implementation of the position algorithm,
using the lasers class which allow to make things easier

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
import csv
from time import time
from lasers import lasersController
from transformations import *
from algorithm_functions import *
from laserpack.msg import Distance
from geometry_msgs.msg import PoseStamped, Accel, TwistStamped, Quaternion, Point
from sensor_msgs.msg import Imu
from threading import Thread
from getch import getch
from Kalman import filter_container


def local_position_callback(data):
    # Input data : local position
    # Output data
    global local_pose
    local_pose = data.pose.position

# update the velocity
def velocity_callback(data):
    # Input data
    global linear_velocity, angular_velocity
    # Output data
    # Publishers
    # Objects
    global filters

    linear_velocity = [filters.Vx.next(data.twist.linear.x),
                       filters.Vy.next(data.twist.linear.y),
                       filters.Vz.next(data.twist.linear.z)]
    angular_velocity = filters.Vyaw.next(data.twist.angular.z)

# update IMU
def imu_callback(data):
    # Input data
    # Output data
    global imu, linearAcceleration
    # Publishers


# Handle lasers
def raw_lasers_callback(data):
    # Input data
    global raw, imu, linear_velocity, angular_velocity, linearAcceleration, local_pose
    # Output data
    global yawprint, last_time_filter
    # Publishers
    global pub_position, pub_filtered, pub_target1, pub_target2
    # Objects
    global lasers, filters

    raw = preCorrectionLasers(data)

    # convert imu to a quaternion tuple
    quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    q = quaternion_from_euler(roll, pitch, 0, axes="sxyz")
    #print rad2degf(roll), rad2degf(pitch), rad2degf(yaw)
    laser1x, orientation1x, laser2x, orientation2x = lasers.preRotateX(q)

    yawMeasured =  getYawInXL(laser1x, orientation1x, raw[0], lasers.X1.length, laser2x, orientation2x, raw[1], lasers.X2.length)

    q = quaternion_from_euler(roll, pitch, yawMeasured, axes="sxyz")

    #
    target = lasers.target(q, raw)

    ### Avoid "too short" measures for Z lasers
    # if(target[4][2] < 0.3 or target[5][2] < 0.3):
    #     if target[4][2] < target[5][2] :
    #         target[5] = target[4]
    #     else :
    #         target[4] = target[5]
    # ###

    yawprint=(rad2degf(yawMeasured), rad2degf(yaw))

    # target[i][j]
    # i = index of the laser extrapolated (0 => 5)
    # j = direction X, Y or Z
    msg = PoseStamped()
    msg.pose.position.x = float(target[0][0] + target[1][0])/2
    if(lasers.count == 4) :
        msg.pose.position.y = float(target[2][1])
        msg.pose.position.z = float(target[3][2])
    else :
        msg.pose.position.y = target[2][1]
        #msg.pose.position.y = float(target[2][1] + target[3][1])/2
        msg.pose.position.z = float(target[4][2] + target[5][2])/2
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    pub_position.publish(msg)

    # Kalman like filtering
    Measurements = ([target[0][0], target[1][0]], \
                   [target[2][1], target[3][1]], \
                   [target[4][2], target[5][2]], \
                   yawMeasured)

    Velocity = linear_velocity + [angular_velocity]

    now = time()
    dt = now - last_time_filter
    last_time_filter = now

    filters.filter_position(Measurements, dt, Velocity, [1,1,1,1])

    pub_filtered.publish(msg)

    last_time_filter = time()


def preCorrectionLasers(data):
    # Objects
    global lasers, filters
    # TODO Handle data, keep old data if outlier/read error detected
    deoffset = list()
    for i in range(lasers.count):
        # Divide by 100 => centimeters to meters
        deoffset.append((data.lasers[i]*lasers.list[i].offset[0] + lasers.list[i].offset[1])/100)

    deoffset = filters.filter_raw(deoffset)
    return deoffset

# init should get values of position of the lasers
def init():
    # Input data
    # Output data
    global raw, yawprint, linear_velocity, angular_velocity, last_time_filter, \
           last_time_acceleration, linearAcceleration, gravity, imu, local_pose
    # Publishers
    # Objects
    global lasers, filters

    # Global variable initialisation
    linearAcceleration =[0,0,0]
    gravity = [0,0,9.81]
    linear_velocity = [0,0,0]
    angular_velocity = 0.0
    yawprint = (0,0)
    raw = Distance()
    imu = Imu()
    imu.orientation.w = 1
    local_pose = Point()

    last_time_acceleration = time()
    last_time_filter = time()

    lasers = lasersController()
    filters = filter_container()


# listerners listen to ROS
def subscribers():
    # Input data
    # Output data
    # Publishers
    global pub_position, pub_filtered
    # Objects

    # Node initiation
    rospy.init_node('position_algorithm')

    pub_position    = rospy.Publisher('lasers/pose', PoseStamped, queue_size=1)
    pub_filtered    = rospy.Publisher('lasers/filtered', PoseStamped, queue_size=1)

    imu_sub         = rospy.Subscriber('mavros/imu/data', Imu, imu_callback)
    state_sub       = rospy.Subscriber('lasers/raw', Distance, raw_lasers_callback)
    velocity_sub    = rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_position_callback)
    velocity_sub    = rospy.Subscriber('mavros/local_position/velocity', TwistStamped, velocity_callback)


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
