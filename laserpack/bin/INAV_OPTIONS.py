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