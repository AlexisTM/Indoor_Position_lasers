#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
tasks.py

This is a task implementation initialy used to control an UAV

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
import time
from math import fabs
from threading import Thread
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from transformations import *
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool

class UAV:
    def __init__(self, setpoint_rate=10):
        self.flying = True
        self.state = State()
        self.position = (0.0,0.0,0.0)
        self.laser_position = Pose
        self.yaw = 0.0
        self.local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.laser_position_sub = rospy.Subscriber('/lasers/filtered', PoseStamped, self.laser_position_sender)
        self.state_subscriber = rospy.Subscriber('mavros/state', State, self.state_callback)
        rospy.wait_for_service('mavros/cmd/arming')
        self.arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        
        self.setpoint_init()
        self.setpoint_rate = rospy.Rate(setpoint_rate)
        self.setpoint_subscriber = rospy.Subscriber('/UAV/Setpoint', PoseStamped, self.setpoint_callback)
        self.setPointsCount = 0


        self.setpoint_thread = Thread(target=self.setpoint_sender).start()
        self.position_thread = Thread(target=self.position_sender).start()

    def setpoint_init(self):
        # type_mask
        # 2552 : XYZ & yaw - POSITION
        # 7104 : XYZ, yaw, vXYZ, TAKE_OFF_SETPOINT
        # 3064 : 0000 1001 1111 1000
        self.setpoint = PositionTarget()
        self.setpoint.coordinate_frame = FRAME_LOCAL_NED
        self.setpoint.type_mask = 2552
        self.setpoint.position.x = 0.0
        self.setpoint.position.y = 0.0
        self.setpoint.position.z = 0.0
        self.setpoint.yaw = 0.0

    def local_position_callback(self, topic):
        self.position = (topic.pose.position.x, topic.pose.position.y, topic.pose.position.z)
        q = (topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w)
        _, _, yaw = euler_from_quaternion(q, axes="sxyz")
        self.yaw = yaw

    def state_callback(self, state):
        self.state = state

    def setpoint_callback(self, setpoint):
        self.setpoint = setpoint

    def laser_position_sender(self, data):
        local_pos_pub = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1)
        while(self.flying):
            data.header.stamp = rospy.Time.now()
            data.header.seq=self.positionCount
            self.positionCount = self.positionCount + 1
            local_pos_pub.publish(data)

    def setpoint_sender(self):
        setpoint_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        while(self.flying):
            self.setpoint = PoseStamped()
            self.setpoint.header.stamp = rospy.Time.now()
            self.setpoint.header.seq=self.setPointsCount
            self.setPointsCount = self.setPointsCount + 1
            setpoint_publisher.publish(self.setpoint)
            self.setpoint_rate.sleep()

    def arm(self, arming_state):
        self.arming_client(arming_state)

    def set_offboard(self):
        self.set_mode_client(custom_mode = "OFFBOARD")

class taskController:
    """ The task controller handle a list with every tasks """
    def __init__(self, rate=10, setpoint_rate=10):
        self.tasks = list()
        self.count = 0
        self.current = 0
        self.setRate(rate)
        self.UAV = UAV(setpoint_rate=setpoint_rate)
	    

    def __str__(self):
        controller_string = "Task Controller :\n"
        for task in self.tasks: 
            controller_string += task.__str__()
            controller_string += "\n"

    def addTask(self, task):
        self.tasks.append(task)
        self.count += 1

    def addTasks(self, tasks):
        for task in tasks:
            self.tasks.append(task)
        self.count += len(tasks)

    def getTasks(self):
        return self.tasks
    
    def getTask(self, index):
        index = index - 1 
        return -1 if (index >= self.count) else self.tasks[index]

    def getCurrentTask(self):
        return self.tasks[current];

    def runTask(self):
        if self.current == -1 :
            rospy.loginfo("No task running (-1)")
        else :
            rospy.loginfo("Running task {0}/{1} - {2}".format(self.current+1, self.count+1, self.tasks[self.current]))

    def setRate(self, rate):
        self.rate = rospy.Rate(rate)

    def spinOnce(self):
        if self.current < self.count :
            task = self.tasks[self.current]
            result = task.run(self.UAV)
            print result
            if result: # returns True if done
                self.current = self.current + 1

        print self.current, self.count
        return 

class task:
    """The Task class defines a class & the needed methods to 
    be compatible with the taskController"""
    def __init__(self, Type, name):
        self.name = name
        self.Type = Type
        self.done = False

    def isDone(self):
        # check if task is ended
        # return True if done, False if not done
        return True
       
    def run(self, UAV):
       
        # do something then return isDone
        return self.isDone()

    def getType(self):
        return self.Type

    def __str__(self):
        return "Task {0} - {1}".format(self.type, self.name)



class target(task, object):
    """The target class is a task. It says to the UAV to go to 
    the target""" 
    def __init__(self, name, x, y, z, yaw, precisionXY = 0.05, precisionZ = 0.05, precisionYAW = 1):
        super(target, self).__init__("target", name)
        self.target       = (x,y,z)
        self.orientation  = yaw
        self.precisionXY  = precisionXY
        self.precisionZ   = precisionZ
        self.precisionYAW = precisionYAW
        self.sent         = False
        self.quaternion   = quaternion_from_euler(0, 0, self.orientation)

    def __str__(self):
        return super(target, self).__str__(self) + "{0} pointing {1} radians".format(self.target, self.orientation)


    def run(self, UAV):
        # Send the position once to the SENDER thread then only check if arrived 
        if not(self.sent) :
            sender = rospy.Publisher('/UAV/Setpoint', PoseStamped, queue_size=1)
            msg = PoseStamped()
            msg.pose.position.x = self.target[0]
            msg.pose.position.y = self.target[1]
            msg.pose.position.z = self.target[2]
            msg.pose.orientation.x = self.quaternion[0]
            msg.pose.orientation.y = self.quaternion[1]
            msg.pose.orientation.z = self.quaternion[2]
            msg.pose.orientation.w = self.quaternion[3]
            sender.publish(msg)
            self.sent = True
        
        return self.isDone(UAV)

    def isDone(self, UAV):
        """ Method to know if the UAV arrived at his position or not

        Better solution for more complex surfaces : 
        http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not/2752753#2752753 """
        if(fabs(UAV.position[0] - self.target[0]) > self.precisionXY):
            return False
        if(fabs(UAV.position[1] - self.target[1]) > self.precisionXY):
            return False
        if(fabs(UAV.position[2] - self.target[2]) > self.precisionZ):
            return False
        if(fabs(UAV.yaw - self.yaw) > self.precisionYAW):
            return False
        # it is done
        return True


""" To be implemented """
class grab(task, object):
    """The grab class is a task. It says to the UAV to grab or realease
    the pliers"""
    def __init__(self, name, state):
        super(grab, self).__init__("grab", name)
        self.state = 0;

    def __str__(self):
        return super(grab, self).__str__()

    def run(self, UAV):
        return self.isDone()

    def isDone(self):
        """ Method to know if the UAV arrived at his position or not
        http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not/2752753#2752753 """
        return True

# Sleeping the right time
class loiter(task, object):
    """The loiter class is a task. It is just a waiting task"""
    def __init__(self, name, waitTime):
        self.waitTime = rospy.Rate(1.0/waitTime)
        self.last = None
        super(loiter, self).__init__("loiter", name)

    def __str__(self):
        return super(loiter, self).__str__()

    def run(self, UAV):
        if(self.last == None):
            self.last = time.time()
        return self.isDone()

    def isDone(self):
        waitTime.sleep()
        return True


# Waiting testing time 
class test(task, object):
    """The test class is a task. It is just a testing task"""
    def __init__(self, name, waitTime):
        super(test, self).__init__("test", name)
        self.waitTime = waitTime
        self.last = None

    def __str__(self):
        return super(test, self).__str__()

    def run(self, UAV):
        if(self.last == None):
            self.last = time.time()
            print("first time")
        return self.isDone()

    def isDone(self):
        now = time.time()
        print(now - self.last)

        #Simplify by using : 
        # return (now - self.last) > self.waitTime

        if(now - self.last > self.waitTime):
            print("done")
            return True
        else :
            print("waiting")
            return False
        
# Arm the pixHawk
class init_UAV(task, object):
    """The test class is a task. It is just a testing task"""
    def __init__(self, name, timeout = 1):
        self.timeout = rospy.Rate(1.0/timeout)
        self.last = None
        super(init_UAV, self).__init__("INIT_UAV", name)

    def __str__(self):
        return super(test, self).__str__()

    def run(self, UAV):
        if not UAV.state.armed :
            UAV.arm(True)
        self.timeout.sleep()
        if not (UAV.state.mode == "OFFBOARD" or UAV.state.mode == "AUTO.LAND"):
            UAV.set_offboard()
        self.timeout.sleep()
        return self.isDone(UAV)

    def isDone(self, UAV):
        return UAV.state.armed and ( UAV.state.mode == "OFFBOARD" or UAV.state.mode == "AUTO.LAND")