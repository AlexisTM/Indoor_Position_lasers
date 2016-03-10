"""
algorithm.py

This script is the Python implementation of the position algorithm.

4 lasers :
It gets the yaw, then correct positions 

6 lasers : 
Add some means to measurments to have better results.

"""

from __future__ import division
import rospy
import mavros
import tf
from transformations import *
from algorithm_functions import *
from laserpack.msg import distance
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from threading import Thread

# update IMU
def imu_callback(data):
    global imu
    imu = data

# update lasers
# Handle lasers
def raw_lasers_callback(data):
    global raw
    global imu
    global positions
    global orientations
    global laserNumber
    global pub_position
    # preCorrectionLasers is TODO
    raw = preCorrectionLasers(data)

    # convert imu to a quaternion tuple
    quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    q = quaternion_from_euler(roll, pitch, 0, axes="sxyz")

    laser1x = quaternionRotation(positions[0], q)
    orientation1x = quaternionRotation(orientations[0], q)
    laser2x = quaternionRotation(positions[1], q)
    orientation2x = quaternionRotation(orientations[1], q)

    yawMeasured =  getYawInXL(laser1, orientation1, raw.lasers[0], orientation_lengths[0], laser2, orientation2, raw.lasers[1], orientation_lengths[1])

    q = quaternion_from_euler(roll, pitch, yawMeasured, axes="sxyz")

    # Rotate ALL vectors
    local_positions = list()
    local_orientations = list()
    local_targets = list()
    for i in range(laserNumber):
        local_positions.append(quaternionRotation(positions[i], q))
        local_orientations.append(quaternionRotation(orientations[i], q))
        local_targets = extrapolate(positions[i], orientations[i], raw)


    # TODO Add an output filter (Kalman filter on positions and yaw)
    msg = PoseStamped()
    msg.pose.position.x = (local_targets[0][0] + local_targets[1][0])/2
    if(laserNumber == 4) :
        msg.pose.position.y = local_targets[2][1]
        msg.pose.position.z = local_targets[3][2]
    else : 
        msg.pose.position.y = (local_targets[2][1] + local_targets[3][1])/2
        msg.pose.position.z = (local_targets[3][2] + local_targets[4][2])/2
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    pub_position(msg)


def preCorrectionLasers():
    global raw
    global lasers_offsets
    global laserNumber
    # TODO Handle data, keep old data if outlier/readerror detected
    # TODO Then, edit raw with correct values
    # TODO Eventual resize data (/100)
    deoffset = list()
    for i in range(laserNumber):
        deoffset.append(raw.lasers[i]*offset[i][0] + offset[i][1])
    return deoffset

# init should get values of posiion of the lasers
def init():
    global positions
    global orientations
    global laserNumber
    global orientation_lengths
    global lasers_offsets
    rospy.init_node('position_algorithm')
    laserNumber = 4
    positions = ((-50,10,20), (50,10,20), (50,-10,20), (-50,-10,20))
    orientations = ((1,0,0), (1,0,0), (0,1,0), (0,0,1))
    orientation_lengths = (length(orientations[0]), length(orientations[1]), length(orientations[2]), length(orientations[3]))
    lasers_offsets =   [[1,-4.45], \
                        [1,-1.489], \
                        [1,0], \
                        [1,0]]
	

# listerners listen to ROS
def subscribers():
    global pub_position
    imu_sub         = rospy.Subscriber('mavros/imu/data', Imu, imu_callback)
    state_sub       = rospy.Subscriber('lasers/raw', distance, raw_lasers_callback)
    pub_position    = rospy.Publisher('lasers/pose', PoseStamped, queue_size=3)
	

def main():
    while not rospy.is_shutdown(): 
        raw_input("Press anything to quit")
        # rospy.spin()
        break

if __name__ == '__main__':
    rospy.loginfo("Position algorithm started")
    try:
        init()
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass