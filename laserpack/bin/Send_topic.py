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


def sendSetpoint():
    global ack_send
    global run
    global setPointsCount
    local_setpoint_pub = rospy.Publisher('/Ack', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while run:
        msg = PoseStamped()
        msg.pose.position.x = 1
        msg.pose.position.y = 2
        msg.pose.position.z = 3
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_setpoint_pub.publish(msg)
        setPointsCount = setPointsCount + 1
        rate.sleep()




def InterfaceKeyboard():
    global setPointsCount
    global run
  

    what = getch()
    if what == "m":
        run = False
        time.sleep(1)
        exit()

    
    rospy.loginfo("Ack sent : %s", setPointsCount )


def init(): 
   
  
    global setPointsCount
    global ack_pub    
    global run 
    setPointsCount = 0
    run = True
    
    rospy.init_node('send_pose')
    tSetPoints = Thread(target=sendSetpoint).start()

    
    while not rospy.is_shutdown(): 
        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()

    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
