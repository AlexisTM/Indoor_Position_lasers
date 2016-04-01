#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
send_setpoint.py

This script sends positions to control the UAV in X, Y, Z

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

Originaly published by Vladimir Ermakov (c) 2015 under GNU GPLv3
Copyright (c) Alexis Paques 2016
Copyright (c) Nabil Nehri 2016
"""
 
import rospy
import mavros
import time
import tf
import numpy as np
from getch import *
from threading import Thread
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros.utils import *
from algorithm_functions import rad2degf


# Callbacks
def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data

def laser_callback(data):
    global laserposition
    laserposition = data

def sendSetpoint():
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global setPointsCount
    global run
    setPointsCount = 0
    local_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while run:
        msg = PoseStamped()
        msg.pose.position.x = float(xSetPoint)
        msg.pose.position.y = float(ySetPoint)
        msg.pose.position.z = float(zSetPoint)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_setpoint_pub.publish(msg)
        setPointsCount = setPointsCount + 1
        rate.sleep()

# In case of use
def sendPosition():
    global laserposition
    global run
    global laser_position_count
    global local_pos_pub
    rate = rospy.Rate(10)
    while run:
	msg = PoseStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.seq=laser_position_count
        msg.pose.position.x = 1
        msg.pose.position.y = float(2.0)
        msg.pose.position.z = 3.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_pos_pub.publish(msg)
        laser_position_count = laser_position_count + 1
        rate.sleep()
        
        #msg = PoseStamped()
	#msg.header.stamp = rospy.Time.now()
	#msg.header.seq=laser_position_count
        #msg.pose.position.x = float(laserposition.pose.position.x)
        #msg.pose.position.y = float(laserposition.pose.position.y)
        #msg.pose.position.z = float(laserposition.pose.position.z)
        #msg.pose.orientation.x = float(laserposition.pose.orientation.x)
        #msg.pose.orientation.y = float(laserposition.pose.orientation.y)
        #msg.pose.orientation.z = float(laserposition.pose.orientation.z)
        #msg.pose.orientation.w = float(laserposition.pose.orientation.w)
        
	#laserposition.header.stamp = rospy.Time.now()
	#laserposition.header.seq=laser_position_count
        #local_pos_pub.publish(laserposition)
        #laser_position_count = laser_position_count + 1
        #rate.sleep()

def InterfaceKeyboard():
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global pose
    global disarm
    global arming_client
    global set_mode_client
    global run
    global laser_position_count

    what = getch()
    if what == "z":
        xSetPoint = xSetPoint + 0.1
    if what == "s":
        xSetPoint = xSetPoint - 0.1
    if what == "q":
        ySetPoint = ySetPoint + 0.1
    if what == "d":
        ySetPoint = ySetPoint - 0.1
    if what == "u":
        zSetPoint = zSetPoint + 0.1
    if what == "j":
        zSetPoint = zSetPoint - 0.1
    if what == "q":
        arming_client(False)
    if what == "a":
        arming_client(True)
    if what == "e":
        set_mode_client(custom_mode = "OFFBOARD")
    if what == "m":
        run = False
        time.sleep(1)
        exit()

    Q = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(Q)
    
    rospy.loginfo("Positions sent : %i", laser_position_count )
    rospy.loginfo("Position x: %s y: %s z: %s", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    rospy.loginfo("Setpoint is now x:%s, y:%s, z:%s", xSetPoint, ySetPoint, zSetPoint)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", rad2degf(euler[0]))
    rospy.loginfo("pitch : %s", rad2degf(euler[1]))
    rospy.loginfo("yaw : %s", rad2degf(euler[2]))


def init(): 
    global state
    global disarm
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global setPointsCount
    global PositionsCount
    global arming_client
    global set_mode_client
    global run 
    global local_pos_pub
    global laser_position_count
    global laserposition
    laserposition = PoseStamped()
    
    laser_position_count = 0
    run = True
    setPointsCount = 0
    PositionsCount = 0
    xSetPoint = 0
    ySetPoint = 0
    zSetPoint = 0
    state = State()
    disarm = False

    rospy.init_node('laserpack_control')
    
    pose_sub  = rospy.Subscriber('mavros/local_position/pose', PoseStamped, Pose_Callback)
    pose_sub  = rospy.Subscriber('lasers/pose', PoseStamped, laser_callback)
    state_sub = rospy.Subscriber('mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    local_pos_pub   = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1)
    # mavros/setpoint_position/local
    # mavros/mocap/pose

    #tSetPoints = Thread(target=sendSetpoint).start()
    # In case we want to send positions
    tPositions = Thread(target=sendPosition).start()
    
    while not rospy.is_shutdown(): 
        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
