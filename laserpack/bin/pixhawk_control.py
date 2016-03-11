#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
pixhawk_control.py

PixHawk_control is a way to send the position and setpoints to the 
PixHawk to test in Gazebo or in real life

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
import numpy as np
from laserpack.getch import *
from threading import Thread
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros.utils import *

# From http://wiki.ros.org/mavros : 
# Published topic = Mavros publish it
# Subscribed topic = Mavros subscribe it
# local : We write on local to send
# pose  : Mavros write on it, comming from PixHawk

# 'local_position', 'pose' = Position drone que lui pense
# 'local_position', 'local' = Position que l'on envoie
# Envoie la hauteur actuelle
def sendSetpoint():
    global zSetPoint
    global setPointsCount
    setPointsCount = 0
    local_setpoint_pub   = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = zSetPoint
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_setpoint_pub.publish(msg)
        rate.sleep()
        setPointsCount = setPointsCount + 1

def sendPosition():
    global zPosition
    global PositionsCount
    #local_pos_pub   = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=10)
    local_pos_pub   = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = float(zPosition)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_pos_pub.publish(msg)
        rate.sleep()
        PositionsCount = PositionsCount+ 1

def True_Position_Callback(pose):
    global local_true_pos_pub
    local_true_pos_pub.publish(pose)


def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data

def IMU_Callback(data):
    global imu
    imu = data

def InterfaceKeyboard():
    global zSetPoint
    global zPosition
    global pose
    global imu
    global disarm
    global arming_client
    global set_mode_client

    what = getch()
    if what == "z":
        zPosition = float(float(zPosition) + 0.1)
    if what == "s": 
        zPosition = float(float(zPosition) - 0.1)
    if what == "o":
        zSetPoint = zSetPoint + 0.1
    if what == "l":
        zSetPoint = zSetPoint - 0.1
    if what == "q":
        arming_client(False)
    if what == "a":
        arming_client(True)
    if what == "e":
        set_mode_client(custom_mode = "OFFBOARD")
    if what == "m":
        exit()

    Q = (
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z,
        imu.orientation.w)
    euler = tf.transformations.euler_from_quaternion(Q)
    
    rospy.loginfo("Positions sent : %i, Setpoints sent : %i",PositionsCount, setPointsCount )
    rospy.loginfo("Manual position %s", zPosition)
    rospy.loginfo("Position     is %s", pose.pose.position.z)
    rospy.loginfo("Setpoint is now %s", zSetPoint)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", euler[0])
    rospy.loginfo("pitch : %s", euler[1])
    rospy.loginfo("yaw : %s", euler[2])


def init(): 
    global state
    global disarm
    global zSetPoint
    global zPosition
    global setPointsCount
    global PositionsCount
    global local_true_pos_pub
    global arming_client
    global set_mode_client
    setPointsCount = 0
    PositionsCount = 0
    zSetPoint = 0
    zPosition = float(0)
    state = State()
    disarm = False

    rospy.init_node('laserpack_main')

    rate = rospy.Rate(20.0)

    pose_sub        = rospy.Subscriber('mavros/local_position/pose', PoseStamped, Pose_Callback)
    imu_sub       = rospy.Subscriber('mavros/imu/data', Imu, IMU_Callback)
    state_sub       = rospy.Subscriber('mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    local_true_pos_pub   = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)
    state_sub       = rospy.Subscriber('lasers/pose', PoseStamped, True_Position_Callback)
    

    tSetPoints = Thread(target=sendSetpoint).start()
    #tPositions = Thread(target=sendPosition).start()
    
    while not rospy.is_shutdown(): 
        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
