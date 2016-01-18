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

# From http://wiki.ros.org/mavros : 
# Published topic = Mavros publish it
# Subscribed topic = Mavros subscribe it
# local : We write on local to send
# pose  : Mavros write on it, comming from PixHawk

# 'local_position', 'pose' = Position drone que lui pense
# 'local_position', 'local' = Position que l'on envoie
# Envoie la hauteur actuelle
def sendSetpoint():
    global zSetPoint
    global setPointsCount
    setPointsCount = 0
    local_setpoint_pub   = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = zSetPoint
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_setpoint_pub.publish(msg)
        rate.sleep()
        setPointsCount = setPointsCount + 1

def sendPosition():
    global zPosition
    global PositionsCount
    local_pos_pub   = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = float(zPosition)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        local_pos_pub.publish(msg)
        rate.sleep()
        PositionsCount = PositionsCount+ 1

def True_Position_Callback(pose):
    local_true_pos_pub(pose)

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
    global zSetPoint
    global zPosition
    global pose
    global imu
    global disarm
    global arming_client
    global set_mode_client

    what = getch()
    if what == "z":
        zPosition = float(float(zPosition) + 0.1)
    if what == "s": 
        zPosition = float(float(zPosition) - 0.1)
    if what == "o":
        zSetPoint = zSetPoint + 0.1
    if what == "l":
        zSetPoint = zSetPoint - 0.1
    if what == "q":
        arming_client(False)
    if what == "a":
        arming_client(True)
    if what == "e":
        set_mode_client(custom_mode = "OFFBOARD")
    if what == "d":
        set_mode_client(base_mode = "MANUAL")

    Q = (
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z,
        imu.orientation.w)
    euler = tf.transformations.euler_from_quaternion(Q)
    rospy.loginfo("Positions sent : %i, Setpoints sent : %i",PositionsCount, setPointsCount )
    rospy.loginfo("Manual position %s", zPosition)
    rospy.loginfo("Position     is %s", pose.pose.position.z)
    rospy.loginfo("Setpoint is now %s", zSetPoint)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", euler[0])
    rospy.loginfo("pitch : %s", euler[1])
    rospy.loginfo("yaw : %s", euler[2])


def init(): 
    global state
    global disarm
    global zSetPoint
    global zPosition
    global setPointsCount
    global PositionsCount
    global local_true_pos_pub
    global arming_client
    global set_mode_client
    setPointsCount = 0
    PositionsCount = 0
    zSetPoint = 0
    zPosition = float(0)
    state = State()
    disarm = False

    rospy.init_node('laserpack_main')

    rate = rospy.Rate(20.0)

    pose_sub        = rospy.Subscriber('mavros/local_position/pose', PoseStamped, Pose_Callback)
    imu_sub       = rospy.Subscriber('mavros/imu/data', Imu, IMU_Callback)
    state_sub       = rospy.Subscriber('mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    local_true_pos_pub   = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=10)
    lasers_pose       = rospy.Subscriber('lasers/pose', PoseStamped, True_Position_Callback)
    

    tSetPoints = Thread(target=sendSetpoint).start()
    #tPositions = Thread(target=sendPosition).start()

    time.sleep(5)
    rospy.loginfo("We should have 100 setpoints sent now")


    #while(state.mode != "OFFBOARD"):
    #    time.sleep(1)
    #    try : 
    #        rospy.loginfo("Trying to set to OFFBOARD mode")
    #        set_mode_client(custom_mode = "OFFBOARD")
    #    except rospy.ServiceException as ex:
    #        rospy.loginfo("OFFBOARD FAILED")
            

    #rospy.loginfo("OFFBOARD SUCCESS")
    #time.sleep(1)

    #count = 0
    #while(state.armed != True):
    #    time.sleep(10)
    #    if count == 10 : 
    #        exit()
    #    try : 
    #        rospy.loginfo("Trying to ARM")
    #        arming_client(True)
    #        count = count + 1
    #    except rospy.ServiceException as ex:
    #        rospy.loginfo("ARM FAILED")


    #rospy.loginfo("ARM SUCCESS")
    while not rospy.is_shutdown(): 
        
        #if disarm:
        #    arming_client(False)

        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass