#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
control_tasks.py

This script sends positions to control the UAV in X, Y, Z using the 
tasks controller

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
from getch import *
from threading import Thread
from geometry_msgs.msg import PoseStamped
from mavros.utils import *
from tasks import *
import sys


def interface_getch():
    global xPosition
    global yPosition
    global zPosition
    global Controller
    global stop

    what = getch()

    if what == "z":
        xPosition = xPosition + 0.1
    if what == "s":
        xPosition = xPosition - 0.1
    if what == "q":
        yPosition = yPosition + 0.1
    if what == "d":
        yPosition = yPosition - 0.1
    if what == "u":
        zPosition = zPosition + 0.1
    if what == "j":
        zPosition = zPosition - 0.1
    if what == "m":
        stop = True
        sys.exit("SHUTDOWN")

def main():
    do_job_thread = Thread(target=do_job).start()
    while not rospy.is_shutdown(): 
        interface_getch()


def do_job():
    global Controller
    global stop
    while not stop :
        Controller.rate.sleep()
        Controller.spinOnce()

def task_feeder():
    global Controller

    Controller.addTask(init_UAV("Init", timeout=1))
    Controller.addTask(target("Position1", 1,1,1,0))
    Controller.addTask(loiter("WaitABit", 10))
    Controller.addTask(target("ComeBack", 0.5, 0.5, 0.5, 0))

def init(): 
    global xPosition
    global yPosition
    global zPosition
    global Controller
    global stop

    stop = False
    xPosition, yPosition, zPosition = (0,0,0)
    rospy.init_node('laserpack_control')
    Controller = taskController(rate=3, setpoint_rate=10)

if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
        task_feeder()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass