#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
algorithm_class.py

This script is the Python implementation of the position algorithm, 
using the lasers class which allow to make things easier

It converts 4 or 6 readings into positions and the yaw, filtering 
values via a Kalman Filter

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

from __future__ import division
from lasers import lasersController
from transformations import *
from algorithm_functions import *
from unittest import *

# Handle lasers
def raw_lasers_callback(data):
    global raw
    global imu
    global lasers
    global pub_position
    global yawprint
    lasers = lasersController()
    # preCorrectionLasers is TODO
    raw = preCorrectionLasers(data)
    
    # convert imu to a quaternion tuple
    quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    q = quaternion_from_euler(roll, pitch, 0, axes="sxyz")
    #print rad2degf(roll), rad2degf(pitch), rad2degf(yaw)
    laser1x, orientation1x, laser2x, orientation2x = lasers.preRotateX(q)
    
    # Yaw of the PixHawk is the OPPOSITE of this
    yawMeasured =  -getYawInXL(laser1x, orientation1x, raw[0], lasers.X1.length, laser2x, orientation2x, raw[1], lasers.X2.length)
    #print "yaw :", rad2degf(yawMeasured)
    

    q = quaternion_from_euler(roll, pitch, yawMeasured, axes="sxyz")

    target = lasers.target(q, raw)
    
    #print "raw : x", raw[0], raw[1]
    #print "raw : y", raw[2], raw[3]
    #print "raw : z", raw[4], raw[5]

    #print "X1:", target[0][0], "X2 :", target[1][0]
    #print "Y1:", target[2][1], "Y2 :", target[3][1]
    #print "Z1:", target[4][2], "Z2 :", target[5][2]
    
    #print "roll:",rad2degf(roll), "pitch:",rad2degf(pitch), "yaw:",rad2degf(yaw)

    yawprint=(rad2degf(yawMeasured), rad2degf(yaw))

    # target[i][j]
    # i = index of the laser extrapolated (0 => 5)
    # j = direction X, Y or Z
    # TODO Add an output filter (Kalman filter on positions and yaw)
    msg = PoseStamped()
    msg.pose.position.x = float(target[0][0] + target[1][0])/2
    if(lasers.count == 4) :
        msg.pose.position.y = float(target[2][1])
        msg.pose.position.z = float(target[3][2])
    else : 
        msg.pose.position.y = float(target[2][1] + target[3][1])/2
        msg.pose.position.z = float(target[4][2] + target[5][2])/2
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])
    
    pub_position.publish(msg)



def testAlgorithm1(cls):
    lasers = lasersController()
    roll = 42
    pitch = -26
    # in Geogebra : distance G'1Q
    distance1 = 18.69
    # in Geogebra : distance F'1P
    distance2 = 18.12

    q = quaternion_from_euler(deg2radf(roll), deg2radf(pitch), 0, axes="sxyz")
    laser1x, orientation1x, laser2x, orientation2x = lasers.preRotateX(q)
    yawMeasured =  -getYawInXL(laser1x, orientation1x, distance1, lasers.X1.length, laser2x, orientation2x, distance2, lasers.X2.length)
    print rad2degf(yawMeasured)

def testPrerotate(cls):
    lasers = lasersController()
    roll = 42
    pitch = -26

    q = quaternion_from_euler(deg2radf(roll), deg2radf(pitch), 0, axes="sxyz")
    laser1x, orientation1x, laser2x, orientation2x = lasers.preRotateX(q)
    # assertEqual(laser1x, (-0.18045368722211999, 0.17539418934983317, 0.027809083232932477))
    # assertEqual(laser2x, (0.13754529898079018, -0.12929518909589841, -0.12232761260214682))
    # assertEqual(orientation1x, (0.89879404629916704, -2.7755575615628914e-17, 0.43837114678907751))
    # assertEqual(orientation2x, (0.89879404629916704, -2.7755575615628914e-17, 0.43837114678907751))

def testTarget(cls):
    pass

if __name__ == '__main__':
    #testPrerotate(1)
    testAlgorithm1(1)

