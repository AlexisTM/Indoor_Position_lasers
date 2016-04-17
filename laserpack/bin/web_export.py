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
from getch import *
from threading import Thread
from laserpack.msg import Report, RPYPose, Battery, Distance
from geometry_msgs.msg import PoseStamped, Point, Vector3, TwistStamped
from std_msgs.msg import Header
from mavros_msgs.msg import BatteryStatus, State, PositionTarget
from transformations import euler_from_quaternion
from algorithm_functions import rad2degf, deg2radf


def report_sender():
    global report
    report_publisher = rospy.Publisher('/web/report', PoseStamped, queue_size=10)
    rate = rospy.Rate(5.0)
    while run:
        report_publisher.publish(report)
        reports_count = reports_count + 1
        rate.sleep()

### CALLBACKS ###
def setpoint_callback(data):
    global report
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, \
                  data.pose.orientation.z, data.pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    report.setpoint.position = data.pose.position
    report.setpoint.roll = roll
    report.setpoint.roll = pitch
    report.setpoint.roll = yaw

def mocap_callback(data):
    global report
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, \
                  data.pose.orientation.z, data.pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    report.mocap.position = data.pose.position
    report.mocap.roll = roll
    report.mocap.roll = pitch
    report.mocap.roll = yaw

def vision_callback(data):
    global report
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, \
                  data.pose.orientation.z, data.pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    report.vision.position = data.pose.position
    report.vision.roll = roll
    report.vision.roll = pitch
    report.vision.roll = yaw


def local_callback(data):
    global report
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, \
                  data.pose.orientation.z, data.pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    report.local.position = data.pose.position
    report.local.roll = roll
    report.local.roll = pitch
    report.local.roll = yaw


def lasers_callback(data):
    global report
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, \
                  data.pose.orientation.z, data.pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    report.lasers_pose.position = data.pose.position
    report.lasers_pose.roll = roll
    report.lasers_pose.roll = pitch
    report.lasers_pose.roll = yaw


def lasers_filtered_callback(data):
    global report
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, \
                  data.pose.orientation.z, data.pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    report.lasers_filtered.position = data.pose.position
    report.lasers_filtered.roll = roll
    report.lasers_filtered.roll = pitch
    report.lasers_filtered.roll = yaw


def lasers_raw_callback(data):
    global report
    report.lasers_raw = data

def state_callback(data):
    global report 
    report.connected = data.connected
    report.armed = data.armed
    report.guided = data.guided
    report.mode = data.mode

def battery_callback(data):
    global report 
    report.voltage = data.voltage
    report.current = data.current
    report.remaining = data.remaining
    pass

def velocity_callback(data):
    pass

def InterfaceKeyboard():
    global run
    what = getch()
    if what == "q":
        run = False
        time.sleep(1)
        exit()
    rospy.loginfo("Reports sent : %s", reports_count )

def init(): 
    global reports_count
    global run 
    global report
    report = Report()
    reports_count = 0
    run = True
    rospy.init_node('web_reporter')

    tWebReport = Thread(target=report_sender).start()

def subscribers():

    setpoint_position_sub= rospy.Subscriber('mavros/setpoint_position/local', PoseStamped, setpoint_callback)
    mocap_sub            = rospy.Subscriber('mavros/mocap/pose', PoseStamped, mocap_callback)
    vision_sub           = rospy.Subscriber('mavros/vision_pose/pose', PoseStamped, vision_callback)
    local_position_sub   = rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_callback)
    velocity_sub         = rospy.Subscriber('mavros/local_position/velocity', TwistStamped, velocity_callback)
    filtered_sub         = rospy.Subscriber('lasers/filtered', PoseStamped, lasers_filtered_callback)
    pose_lasers          = rospy.Subscriber('lasers/pose', PoseStamped, lasers_callback)
    raw_lasers           = rospy.Subscriber('lasers/raw', Distance, lasers_raw_callback)
    state_sub            = rospy.Subscriber('mavros/state', State, state_callback)
    battery_sub          = rospy.Subscriber('mavros/battery', BatteryStatus, battery_callback)

def main():
    while not rospy.is_shutdown(): 
        InterfaceKeyboard()


if __name__ == '__main__':
    try:
        init()
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
