#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
lasers.py

This class represents the lasers real configuration to be used in the 
algorithm. It allows to avoid to hardcode the positions.

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

from math import sqrt
from transformations import *
from algorithm_functions import *

# offset is defined as 
class laser:
    def __init__(self, position, orientation, offset):
        self.position = position
        self.orientation = orientation
        self.offset = offset
        self.length = sqrt(orientation[0]*orientation[0] + orientation[1]*orientation[1] + orientation[2]*orientation[2])


class lasersController: 
    def __init__(self):
        self.X1 = laser((-15,-21,-4), (1,0,0), (1,-4.45))
        self.X2 = laser((7,20,-4), (1,0,0), (1,-1.489))
        self.Y1 = laser((-31,-2,-4), (0,1,0), (1,0))
        self.Y2 = laser((32,-2,-4), (0,1,0), (1,0))
        self.Z1 = laser((-13,-21,-7.5), (0,0,1), (1,0))
        self.Z2 = laser((10,20,-7.5), (0,0,1), (1,0))
        self.list = (X1, X2, Y1, Y2, Z1, Z2)
        self.count = 6

    def target(self, q, raw):
        target = list()
        for laser in self.list:
            position = quaternionRotation(laser.position, q)
            orientation = quaternionRotation(laser.orientation, q)
            target.append(extrapolate(position[i], orientation[i], raw.lasers[i]))
        return target

    def preRotateX(self, q):
        # convert imu to a quaternion tuple
        laser1 = quaternionRotation(self.X1.position, q)
        orientation1 = quaternionRotation(self.X1.orientation, q)
        laser2 = quaternionRotation(self.X2.position, q)
        orientation2 = quaternionRotation(self.X2.orientation, q)
        return (laser1, orientation1, laser2, orientation2)


    def preRotateY(self, q):
        # convert imu to a quaternion tuple
        laser1 = quaternionRotation(self.Y1.position, q)
        orientation1 = quaternionRotation(self.Y1.orientation, q)
        laser2 = quaternionRotation(self.Y2.position, q)
        orientation2 = quaternionRotation(self.Y2.orientation, q)
        return (laser1, orientation1, laser2, orientation2)
