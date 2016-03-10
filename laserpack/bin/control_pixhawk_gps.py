#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae, Vladimir Ermakov
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233
#    - Use mavros.setpoint module for topics
 
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
 
from math import *

 
 
class SetpointPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
 
        # publisher for mavros/setpoint_position/local
    local_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # subscriber for mavros/local_position/local
    self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    PoseStamped, self.reached)
 
    tnavigate = Thread(target=navigate).start()

 
    def navigate(self):
        
        global setPointsCount
        setPointsCount = 0
        #local_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            msg = PoseStamped()
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            local_setpoint_pub.publish(msg)
            rate.sleep()
            setPointsCount = setPointsCount + 1
 
    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
 
        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()
 
        time.sleep(delay)
 
    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.5
 
        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
 
 
def setpoint_demo():
    global state
    global disarm
    global arming_client
    global set_mode_client
    state = State()
    disarm = False

    rospy.init_node('setpoint_position_demo')
    mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10.0)

    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    #rospy.wait_for_service('mavros/set_mode')
    #set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
 
    setpoint = SetpointPosition()
 
    rospy.loginfo("Climb")
    setpoint.set(0.0, 0.0, 3.0, 0)
    setpoint.set(0.0, 0.0, 10.0, 5)
 
    rospy.loginfo("Sink")
    setpoint.set(0.0, 0.0, 8.0, 5)
 
    rospy.loginfo("Fly to the right")
    setpoint.set(10.0, 4.0, 8.0, 5)
 
    rospy.loginfo("Fly to the left")
    setpoint.set(0.0, 0.0, 8.0, 5)
 
    offset_x = 0.0
    offset_y = 0.0
    offset_z = 10.0
    sides = 360
    radius = 20
 
    rospy.loginfo("Fly in a circle")
    setpoint.set(0.0, 0.0, 10.0, 3)   # Climb to the starting height first
    i = 0
    while not rospy.is_shutdown():
        x = radius * cos(i * 2 * pi / sides) + offset_x
        y = radius * sin(i * 2 * pi / sides) + offset_y
        z = offset_z
 
        wait = False
        delay = 0
        if (i == 0 or i == sides):
            # Let it reach the setpoint.
            wait = True
            delay = 5
 
        setpoint.set(x, y, z, delay, wait)
 
        i = i + 1
        rate.sleep()
 
        if (i > sides):
            rospy.loginfo("Fly home")
            setpoint.set(0.0, 0.0, 10.0, 5)
            break
 
    # Simulate a slow landing.
    setpoint.set(0.0, 0.0,  8.0, 5)
    setpoint.set(0.0, 0.0,  3.0, 5)
    setpoint.set(0.0, 0.0,  2.0, 2)
    setpoint.set(0.0, 0.0,  1.0, 2)
    setpoint.set(0.0, 0.0,  0.0, 2)
    setpoint.set(0.0, 0.0, -0.2, 2)
 
    rospy.loginfo("Bye!")
 
 
if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
