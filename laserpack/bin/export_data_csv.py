#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
export_data_csv.py

This script writes a csv @frequency of laser positions @~100Hz

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
import csv
import tf
from algorithm_functions import rad2degf
from time import time
from laserpack.msg import distance
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from transformations import *
from threading import Thread

def imuCB(data):
    global imu_orientation
    imu_orientation = (str(data.orientation.x), str(data.orientation.y), str(data.orientation.z), str(data.orientation.w))
    global imu_linear_accel
    imu_linear_accel = (str(data.linear_acceleration.x), str(data.linear_acceleration.y), str(data.linear_acceleration.z))

def velocityCB(data):
    global velocity
    velocity = (str(data.twist.linear.x),str(data.twist.linear.y),str(data.twist.linear.z),str(data.twist.angular.z))

def writingCB(data):
    global writing
    writing = data

def setpointCB(data):
    global setpoint
    setpoint = poseStamped2Array(data)

def positionCB(data):
    global position
    position = poseStamped2Array(data, True)

def lasersposeCB(data):
    global lasers_pose
    lasers_pose = poseStamped2Array(data)

def lasersrawCB(data):
    global lasers_raw
    lasers_raw = distance2Array(data)

def poseStamped2Array(data, RollPitch=False):
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    
    
    if RollPitch:
        return [ str(data.pose.position.x), str(data.pose.position.y), str(data.pose.position.z), str(rad2degf(roll)), str(rad2degf(pitch)), str(rad2degf(yaw))]
    return [ str(data.pose.position.x), str(data.pose.position.y), str(data.pose.position.z), str(rad2degf(yaw))]

def distance2Array(data):
    return [ str(data.lasers[0]), str(data.lasers[1]), str(data.lasers[2]), str(data.lasers[3]), str(data.lasers[4]), str(data.lasers[5])]

def writer():
    global setpoint
    global position
    global lasers_raw
    global lasers_pose
    global writing
    global imu_orientation
    global imu_linear_accel
    global velocity

    rate = rospy.Rate(10)
    
    with open('result.csv', 'w') as csvfile:
        data_writer = csv.writer(csvfile, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        fieldnames = ['time', 'setpoint_x', 'setpoint_y', 'setpoint_z', 'setpoint_yaw', \
                      'local_x', 'local_y', 'local_z', 'local_roll', 'local_pitch', 'local_yaw', \
                      'lasers_x', 'lasers_y', 'lasers_z', 'lasers_yaw', \
                      'raw_x_1', 'raw_x_2', 'raw_y_1', 'raw_y_2','raw_z_1', 'raw_z_2', \
                      'local_vel_x', 'local_vel_y', 'local_vel_z', 'local_vel_yaw', \
                      'accel_lin_x', 'accel_lin_y', 'accel_lin_z' \
                      'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']


        data_writer.writerow(fieldnames)
        initial_time = time()

       #while writing:
        diff_time = 0
        while diff_time < 120:
            diff_time = time()-initial_time
            data_writer.writerow([diff_time] + setpoint + position + lasers_pose + lasers_raw + velocity + imu_linear_accel + imu_orientation)
            rate.sleep()
            print diff_time
	    


def subscribers():
    setpoint_position   = rospy.Subscriber('mavros/setpoint_position/local', PoseStamped, setpointCB)
    local_position      = rospy.Subscriber('mavros/local_position/pose', PoseStamped, positionCB)
    pose_lasers         = rospy.Subscriber('lasers/pose', PoseStamped, lasersposeCB)
    raw_lasers          = rospy.Subscriber('lasers/raw', distance, lasersrawCB)
    writing_sub         = rospy.Subscriber('export/csv/writing', Bool, writingCB)
    velocity_sub        = rospy.Subscriber('mavros/local_position/velocity', TwistStamped, velocityCB)
    imu_sub             = rospy.Subscriber('mavros/imu/data', Imu, imuCB)


def main():
    global setpoint
    global position
    global lasers_raw
    global lasers_pose
    global writing
    global imu_orientation
    global imu_linear_accel
    global velocity

    # init global objects
    writing = False
    setpoint = ['', '', '', '']
    lasers_pose = ['', '', '', '']
    position = ['', '', '', '', '', '',]
    lasers_raw = ['', '', '', '', '', '',]
    imu_orientation = ['', '', '', '']
    imu_linear_accel = ['', '', '']
    velocity = ['', '', '', '']
    
    listener = Thread(target=writer).start()
    
    while not rospy.is_shutdown(): 
        rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Data export started")
    try:
	rospy.init_node('csv_export')
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
