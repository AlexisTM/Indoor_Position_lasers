#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
INAV_OPTIONS.py 

INAV_OPTIONS.py is a way to set parameters for the PixHawk. You can 
just edit the parameters name and value. Care or the "Float" and "Int"
data, indeed, if the data should be a float, then add .0. Else, it will 
be interpreted like an int, sent like an int and used like a float.

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

import rospy

from mavros.utils import *
from mavros.param import *

def paramsInit(): 
	global parameters
	parameters = [
			["ATT_EXT_HDG_M", 		1]
			["CBRK_NO_VISION", 		0], 	\
			["INAV_W_Z_BARO", 		0], 	\
			["INAV_W_Z_GPS_P", 		0], 	\
			["INAV_W_Z_GPS_V", 		0], 	\
			["INAV_W_Z_VIS_P", 		10], 	\
			["INAV_W_Z_LIDAR", 		0], 	\
			["INAV_W_XY_GPS_P", 	0], 	\
			["INAV_W_XY_GPS_V", 	0], 	\
			["INAV_W_XY_VIS_P", 	0], 	\
			["INAV_W_XY_VIS_V", 	0], 	\
			["INAV_W_MOC_P", 		10], 	\
			["INAV_W_XY_FLOW", 		0], 	\
			["INAV_W_XY_RES_V", 	0], 	\
			["INAV_W_GPS_FLOW", 	0], 	\
			["INAV_W_ACC_BIAS", 	0], 	\
			["INAV_FLOW_K", 		0], 	\
			["INAV_FLOW_Q_MIN", 	0], 	\
			["INAV_LIDAR_FILT", 	0], 	\
			["INAV_LIDAR_ERR", 		0], 	\
			["INAV_LAND_T", 		0], 	\
			["INAV_LAND_DISP", 		0], 	\
			["INAV_LAND_THR", 		0], 	\
			["INAV_DELAY_GPS", 		0], 	\
			["INAV_FLOW_DIST_X", 	0], 	\
			["INAV_FLOW_DIST_Y", 	0], 	\
			["INAV_DISAB_MOCAP", 	1], 	\
			["INAV_ENABLED", 		1]		\
			]

def changeAll():
	global parameters
	for parameter in parameters:
		result = param_set(parameter[0], int(parameter[1])
		print "set", parameter[0], "to", parameter[1], "got", result

def main():
	paramsInit()
	changeAll()

if __name__ == '__main__':
    main()