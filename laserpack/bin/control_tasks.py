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


# Callbacks
def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data