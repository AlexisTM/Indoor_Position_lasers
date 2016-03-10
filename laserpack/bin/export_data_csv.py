"""
export_data_csv.py

This script writes a csv @frequency of laser positions @~100Hz
"""

import rospy
import csv
import tf
from algorithm_functions import rad2degf
from time import time
from laserpack.msg import distance
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from transformations import *
from threading import Thread

def writingCB(data):
    global writing
    writing = data

def setpointCB(data):
    global setpoint
    setpoint = poseStamped2Array(data)

def positionCB(data):
    global position
    position = poseStamped2Array(data, True)

def lasersposeCB(data):
    global lasers_pose
    lasers_pose = poseStamped2Array(data)

def lasersrawCB(data):
    global lasers_raw
    lasers_raw = distance2Array(data)

def poseStamped2Array(data, RollPitch=False):
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    
    roll, pitch, yaw = euler_from_quaternion(quaternion, axes="sxyz")
    
    
    if RollPitch:
        return [ str(data.pose.position.x), str(data.pose.position.x), str(data.pose.position.x), str(rad2degf(roll)), str(rad2degf(pitch)), str(rad2degf(yaw))]
    return [ str(data.pose.position.x), str(data.pose.position.y), str(data.pose.position.z), str(rad2degf(yaw))]

def distance2Array(data):
    return [ str(data[0]), str(data[1]), str(data[2]), str(data[3]), str(data[4]), str(data[5])]

def writer():
    global setpoint
    global position
    global lasers_raw
    global lasers_pose
    global writing

    rate = rospy.Rate(10)
    
    with open('result.csv', 'w') as csvfile:
        data_writer = csv.writer(csvfile, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        fieldnames = ['time', 'setpoint_x', 'setpoint_y', 'setpoint_z', 'setpoint_yaw', 'local_x', 'local_y', 'local_z', 'local_roll', 'local_pitch', 'local_yaw', 'lasers_x', 'lasers_y', 'lasers_z', 'lasers_yaw', 'raw_x_1', 'raw_x_2', 'raw_y_1', 'raw_y_2','raw_z_1', 'raw_z_2']
        data_writer.writerow(fieldnames)
        initial_time = time()

       #while writing:
        i = 0
        while i < 100:
	    print 'writing'
            diff_time = time()-initial_time
            data_writer.writerow([diff_time] + setpoint + position + lasers_pose + lasers_raw)
            rate.sleep()
            i = i + 1
	    


def subscribers():
    setpoint_position   = rospy.Subscriber('mavros/setpoint_position/pose', PoseStamped, setpointCB)
    local_position      = rospy.Subscriber('mavros/local_position/pose', PoseStamped, positionCB)
    pose_lasers         = rospy.Subscriber('lasers/pose', PoseStamped, lasersposeCB)
    raw_lasers          = rospy.Subscriber('lasers/raw', distance, lasersrawCB)
    writingCB_sub       = rospy.Subscriber('export/csv/writing', Bool, writingCB)


def main():
    global setpoint
    global position
    global lasers_raw
    global lasers_pose
    global writing

    # init global objects
    writing = False
    setpoint = ['', '', '', '']
    lasers_pose = ['', '', '', '']
    position = ['', '', '', '', '', '',]
    lasers_raw = ['', '', '', '', '', '',]
    
    listener = Thread(target=writer).start()
    
    while not rospy.is_shutdown(): 
        rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Data export started")
    try:
	rospy.init_node('csv_export')
        subscribers()
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
