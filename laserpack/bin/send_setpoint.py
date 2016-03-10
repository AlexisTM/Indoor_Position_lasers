#!/usr/bin/env python
# vim:set ts=4 sw=4 et:

 
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


def sendSetpoint():
    global xSetPoint
    global ySetPoint
    global zSetPoint

    global setPointsCount
    setPointsCount = 0
    local_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.pose.position.x = xSetPoint
        msg.pose.position.y = ySetPoint
        msg.pose.position.z = zSetPoint
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_setpoint_pub.publish(msg)
        rate.sleep()
        setPointsCount = setPointsCount + 1



def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data

def IMU_Callback(data):
    global imu
    imu = data

def InterfaceKeyboard():
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global pose
    global imu
    global disarm
    global arming_client
    global set_mode_client

    what = getch()
    if what == "z":
        xSetPoint = xSetPoint + 0.1
    if what == "s":
        xSetPoint = xSetPoint - 0.1
    if what == "q":
        ySetPoint = ySetPoint + 0.1
    if what == "d":
        ySetPoint = ySetPoint - 0.1
    if what == "u":
        zSetPoint = zSetPoint + 0.1
    if what == "j":
        zSetPoint = zSetPoint - 0.1
    if what == "q":
        arming_client(False)
    if what == "a":
        arming_client(True)
    if what == "e":
        set_mode_client(custom_mode = "OFFBOARD")
    if what == "m":
        exit()

    Q = (
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z,
        imu.orientation.w)
    euler = tf.transformations.euler_from_quaternion(Q)
    
    rospy.loginfo("Setpoints sent : %i", setPointsCount )
    rospy.loginfo("Position     is %s", pose.pose.position.z)
    rospy.loginfo("Setpoint is now x:%s, y:%s, z:%s", xSetPoint, ySetPoint, zSetPoint)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", euler[0])
    rospy.loginfo("pitch : %s", euler[1])
    rospy.loginfo("yaw : %s", euler[2])


def init(): 
    global state
    global disarm
    global xSetPoint
    global ySetPoint
    global zSetPoint
    global setPointsCount
    global PositionsCount
    global arming_client
    global set_mode_client
    setPointsCount = 0
    PositionsCount = 0
    xSetPoint = 0
    ySetPoint = 0
    zSetPoint = 0
    state = State()
    disarm = False

    rospy.init_node('laserpack_main')
    
    pose_sub  = rospy.Subscriber('mavros/local_position/pose', PoseStamped, Pose_Callback)
    imu_sub   = rospy.Subscriber('mavros/imu/data', Imu, IMU_Callback)
    state_sub = rospy.Subscriber('mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    

    tSetPoints = Thread(target=sendSetpoint).start()
    
    while not rospy.is_shutdown(): 
        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass
