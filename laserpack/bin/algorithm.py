"""
algorithm.py

This script is the Python implementation of the position algorithm.

4 lasers :
It gets the yaw, then correct positions 

6 lasers : 
Add some means to measurments to have better results.

"""

from __future__ import division
from transformations import *
from algorithm_functions import *
import mavros
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

# init should get values of posiion of the lasers
def init():
	pass

# listerners listen to ROS
def listeners():
	pass

def main():
	pass

if __name__ == '__main__':
    rospy.loginfo("Position algorithm started")
    try:
        init()
        listeners()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass