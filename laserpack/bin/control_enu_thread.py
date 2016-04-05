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
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool
from mavros.utils import *
from algorithm_functions import rad2degf, deg2radf
from transformations import *


# Callbacks
def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data

def laser_callback(data):
    global laserposition
    global local_pos_pub
    global laser_position_count
    global activeX

    laserposition = data

    msg = PoseStamped()
    msg.header.stamp=rospy.Time.now()
    msg.header.seq=laser_position_count
    # msg.pose.position.x = msg.pose.position.x
    # msg.pose.position.y = msg.pose.position.y
    # msg.pose.position.z = msg.pose.position.z
    msg.pose.position.x = laserposition.pose.position.x
    msg.pose.position.y = laserposition.pose.position.y
    msg.pose.position.z = -laserposition.pose.position.z
    msg.pose.orientation.x = laserposition.pose.orientation.x
    msg.pose.orientation.y = laserposition.pose.orientation.y
    msg.pose.orientation.z = laserposition.pose.orientation.z
    msg.pose.orientation.w = laserposition.pose.orientation.w
    local_pos_pub.publish(msg)
    laser_position_count = laser_position_count + 1

def sendSetpoint():
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global yawSetPoint
    global setPointsCount
    global run
    global pose
    global activeX

    setPointsCount = 0
    local_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/position_ned', PositionTarget, queue_size=1)
    
    rate = rospy.Rate(30)
    
    while run:
        # q = quaternion_from_euler(0, 0, deg2radf(yawSetPoint), axes="sxyz")

        # msg = PoseStamped()
        # msg.header.stamp = rospy.Time.now()
        # msg.header.seq=setPointsCount
        # msg.pose.position.x = float(xSetPoint)
        # msg.pose.position.y = float(ySetPoint)
        # msg.pose.position.z = -float(zSetPoint)
        # msg.pose.orientation.x = q[0]
        # msg.pose.orientation.y = q[1]
        # msg.pose.orientation.z = q[2]
        # msg.pose.orientation.w = q[3]
        # local_setpoint_pub.publish(msg)
        # setPointsCount = setPointsCount + 1
        # rate.sleep()

        msg = PositionTarget()
        msg.coordinate_frame = msg.FRAME_LOCAL_NED
        msg.position.x = float(xSetPoint)
        msg.position.y = float(ySetPoint)
        msg.position.z = -float(zSetPoint)
        msg.yaw = 0.0
        msg.yaw_rate = 1
        local_setpoint_pub.publish(msg)
        setPointsCount = setPointsCount + 1
        rate.sleep()

# # If we want to reduce rate
# def sendPosition():
#     global laserposition
#     global run
#     global laser_position_count
#     global local_pos_pub
#     global activeX
#     rate = rospy.Rate(100)
#     while run:

#         laserposition.header.stamp=rospy.Time.now()
#         laserposition.header.seq=laser_position_count
#         laserposition.pose.position.x = 0.5
#         laserposition.pose.position.y = 1
#         laserposition.pose.position.z = 2.3
#         laserposition.pose.orientation.x = 0
#         laserposition.pose.orientation.y = 0
#         laserposition.pose.orientation.z = 0
#         laserposition.pose.orientation.w = 1

#         local_pos_pub.publish(laserposition)
#         laser_position_count = laser_position_count + 1
#         rate.sleep()

def InterfaceKeyboard():
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global yawSetPoint
    global pose
    global disarm
    global arming_client
    global set_mode_client
    global run
    global laser_position_count
    global activeX

    what = getch()
    if what == "t":
        xSetPoint = xSetPoint + 0.1
    if what == "g":
        xSetPoint = xSetPoint - 0.1
    if what == "f":
        ySetPoint = ySetPoint + 0.1
    if what == "h":
        ySetPoint = ySetPoint - 0.1
    if what == "i":
        zSetPoint = zSetPoint + 0.1
    if what == "k":
        zSetPoint = zSetPoint - 0.1

    if what == "b":
        yawSetPoint = yawSetPoint + 1
    if what == "n":
        yawSetPoint = yawSetPoint - 1

    if what == "c": 
        xSetPoint = pose.pose.position.x
        ySetPoint = pose.pose.position.y
        zSetPoint = pose.pose.position.z

    if what == "p":
        xSetPoint = pose.pose.position.x
        ySetPoint = pose.pose.position.y
        activeX = True
    if what == "l":
        activeX = False
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
    
    rospy.loginfo("MODE X: ")
    rospy.loginfo("true" if activeX else "false")
    rospy.loginfo("Positions sent : %i", laser_position_count )
    rospy.loginfo("Position x: %s y: %s z: %s", pose.pose.position.x, pose.pose.position.y, -pose.pose.position.z)
    rospy.loginfo("Setpoint is now x:%s, y:%s, z:%s", xSetPoint, ySetPoint, -zSetPoint)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", rad2degf(euler[0]))
    rospy.loginfo("pitch : %s", rad2degf(euler[1]))
    rospy.loginfo("yaw : %s", rad2degf(euler[2]))
    rospy.loginfo("wanted yaw : %s", yawSetPoint)

def init(): 
    global state
    global disarm
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global yawSetPoint
    global setPointsCount
    global PositionsCount
    global arming_client
    global set_mode_client
    global run 
    global local_pos_pub
    global laser_position_count
    global laserposition
    global activeX

    # When false, setpoint in XY = position in XY
    activeX = False
    yawSetPoint = 0

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
    
    local_pos_pub   = rospy.Publisher('mavros/mocap/ned', PoseStamped, queue_size=1)
    pose_sub  = rospy.Subscriber('mavros/local_position/pose', PoseStamped, Pose_Callback)
    laser_pose_sub  = rospy.Subscriber('lasers/filtered', PoseStamped, laser_callback)
    state_sub = rospy.Subscriber('mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    # mavros/setpoint_position/local
    # mavros/mocap/pose

    tSetPoints = Thread(target=sendSetpoint).start()
    # In case we want to send positions
    # tPositions = Thread(target=sendPosition).start()
    
    while not rospy.is_shutdown(): 
        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
