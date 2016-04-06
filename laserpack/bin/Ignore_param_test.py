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
from transformations import *
from algorithm_functions import rad2degf, deg2radf



# Callbacks
def State_Callback(data):
    global state
    state = data



def sendSetpoint():
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global yawSetPoint
    global setPointsCount
    global run
    global pose
    global activeIgnore


    setPointsCount = 0
    local_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    rate = rospy.Rate(5.0)
    while run:
        # Generate setpoint yaw

        q = quaternion_from_euler(0, 0, deg2radf(yawSetPoint), axes="sxyz")

        msg = PositionTarget()
        if(activeIgnore):
            #msg.type_mask = msg.IGNORE_PX or msg.IGNORE_PY or msg.IGNORE_VX or msg.IGNORE_VY or msg.IGNORE_VZ or msg.IGNORE.AFX or msg.IGNORE.AFY or msg.IGNORE.AFZ
            msg.type_mask = 2558 # 0b1001 1111 1110
            msg.coordinate_frame = msg.FRAME_LOCAL_NED 
        else : 
           #  msg.type_mask = msg.IGNORE_VX or msg.IGNORE_VY or msg.IGNORE_VZ or msg.IGNORE.AFX or msg.IGNORE.AFY or msg.IGNORE.AFZ
            msg.type_mask = 2552 #0b1001 1111 1000
            msg.coordinate_frame = msg.FRAME_LOCAL_NED
        
        msg.header.stamp = rospy.Time.now()
        msg.header.seq=setPointsCount
        msg.position.x = float(xSetPoint)
        msg.position.y = float(ySetPoint)
        msg.position.z = float(zSetPoint)
        msg.yaw = deg2radf(yawSetPoint)
        local_setpoint_pub.publish(msg)
        setPointsCount = setPointsCount + 1
        rate.sleep()


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
    global activeIgnore

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
        #xSetPoint = pose.pose.position.x
        #ySetPoint = pose.pose.position.y
        activeIgnore = True
    if what == "l":
        activeIgnore = False
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

 
    
    rospy.loginfo("MODE X: ")
    rospy.loginfo("true" if activeIgnore else "false")
    rospy.loginfo("Setpoint is now x:%s, y:%s, z:%s", xSetPoint, ySetPoint, zSetPoint)
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
    global activeIgnore

    # When false, setpoint in XY = position in XY
    activeIgnore = False
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


    rospy.init_node('Ignore_param_test')
    
    state_sub = rospy.Subscriber('mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    local_pos_pub   = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1)
    # mavros/setpoint_position/local
    # mavros/mocap/pose

    tSetPoints = Thread(target=sendSetpoint).start()
    # In case we want to send positions
    
    while not rospy.is_shutdown(): 
        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
