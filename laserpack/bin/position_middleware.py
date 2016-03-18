#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
position_middleware.py

This script aims to capture and maintain the offset between the position
given by the lasers and the PixHawk. Indeed, the PixHawk have a local 
position which is (0,0,0) at the point he started. This script will adapt
setpoints to give the correct value 

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
 
from geometry_msgs.msg import PoseStamped

class position_middleware:
    """
    This class is used to maintain (know) the offset between the local position
    of the pixHawk and the real position from the lasers, mocap or vision. 
    
    It also aims to detect and learn know which is the reference of the PixHawk.
    Indeed, the reference "yaw = 0" is different and we could have to rotate the 
    whole system. 

    Therefore it will be easy to send the correct setpoint.

    The offset tend to be constant => Kalman filter
    """
    def __init__(self):
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.last_local_position = PoseStamped()
        self.last_laser_position = PoseStamped()

    def getOffset(self):
    	return [self.x, self.y, self.z, self.yaw]

   	def getSetpoint(self, local_position, laser_position, setpoint):
   		self.x = local_position.pose.position.x - laser_position.pose.position.x
   		self.y = local_position.pose.position.y - laser_position.pose.position.y
   		self.z = local_position.pose.position.z - laser_position.pose.position.z

   		setpoint.pose.position.x = setpoint.pose.position.x + self.x
   		setpoint.pose.position.y = setpoint.pose.position.y + self.y
   		setpoint.pose.position.z = setpoint.pose.position.z + self.z

   		self.last_local_position = local_position
   		self.last_laser_position = laser_position
   		return setpoint
