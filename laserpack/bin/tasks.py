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
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point, Vector3
from transformations import *
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool

class UAV:
    def __init__(self, setpoint_rate=10):
        self.stopped = False
        self.flying = True

        # Variable initiating
        self.state = State()
        self.local_position = Point()
        self.local_yaw = 0.0
        self.laser_position = Point()
        self.laser_yaw = 0.0

        # Configurations
        self.landing_speed = -0.1 # 0.1 meters/s to go down
        self.landing_altitude = 0.10 # At 0.1 meters, shutdown motors, you are done
        self.takeoff_speed = 0.5 # 0.5 meters/s to get up
        self.takeoff_altitude = 0.50 # 0.5 meters to takeoff, once there, takeoff is succeeded


        # PixHawk position subscriber
        self.local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        
        # State subscriber
        self.state_subscriber = rospy.Subscriber('mavros/state', State, self.state_callback)
        
        # Arming & mode Services
        rospy.wait_for_service('mavros/cmd/arming')
        self.arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Setpoints
        self.setpoint_init()
        self.setpoint_rate = rospy.Rate(setpoint_rate)
        #self.setpoint_subscriber = rospy.Subscriber('/UAV/Setpoint', PoseStamped, self.setpoint_callback)
        self.laser_position_sub  = rospy.Subscriber('/UAV/Position', PoseStamped, self.laser_position_sender)
        self.setPointsCount = 0

        # Senders threads
        self.setpoint_thread = Thread(target=self.setpoint_sender).start()

        # Configurations : 
        self.type_mask_Fly = 2555 # 2552 - 0000 1001 1111 1000, position setpoint + Pxyz Yaw
        self.type_mask_Takeoff = 6599 # 6599 - 0001 1001 1100 0111, Takeoff setpoint + Vxyz Yaw
        self.type_mask_Land = 10695 # 10695 - 0010 1001 1100 0111, Land setpoint + Vxyz Yaw
        # Could be : 
        # self.type_mask_Land = 10688 # 10695 - 0010 1001 1100 0000, Land setpoint + Pxyz Vxyz Yaw
        self.type_mask_Loiter = 14784 # 14784 - 0011 1001 1100 0000, Loiter setpoint + Pxyz Vxyz Yaw

    def setpoint_position(self, position, yaw):
        self.setpoint.type_mask = self.type_mask_Fly
        self.setpoint.velocity = Vector3()
        self.setpoint.position = position
        self.setpoint.yaw = yaw

    def setpoint_takeoff(self):
        self.setpoint.type_mask = self.type_mask_Takeoff
        self.setpoint.velocity = Vector3(0.0,self.takeoff_speed,0.0)

    def setpoint_land(self):
        self.setpoint.type_mask = self.type_mask_Land
        self.setpoint.velocity = Vector3(0.0,self.landing_speed,0.0)

    def setpoint_loiter(self):
        self.setpoint.type_mask = self.type_mask_Loiter
        self.setpoint.velocity = Vector3(0.0,0.0,0.0)

    def setpoint_init(self):
        # type_mask
        # 2552 : XYZ & yaw - POSITION
        # 7104 : XYZ, yaw, vXYZ, TAKE_OFF_SETPOINT
        # 3064 : 0000 1001 1111 1000
        self.setpoint = PositionTarget()
        self.setpoint.coordinate_frame = self.setpoint.FRAME_LOCAL_NED
        self.setpoint.type_mask = self.type_mask_Fly
        self.setpoint.position = Point()
        self.setpoint.yaw = 0.0
        self.setpoint.velocity = Vector3()
        self.setpoint.acceleration_or_force = Vector3()
        self.setpoint.yaw_rate = 0.0

    def local_position_callback(self, local):
        self.local_position = (local.pose.position.x, local.pose.position.y, local.pose.position.z)
        q = (local.pose.orientation.x, local.pose.orientation.y, local.pose.orientation.z, local.pose.orientation.w)
        _, _, yaw = euler_from_quaternion(q, axes="sxyz")
        self.yaw = yaw

    def state_callback(self, state):
        self.state = state

    # def setpoint_callback(self, setpoint):
    #     self.setpoint = setpoint

    def laser_position_sender(self, data):
        local_pos_pub = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=1)
        while(not self.stopped):
            data.header.stamp = rospy.Time.now()
            data.header.seq=self.positionCount
            self.positionCount = self.positionCount + 1
            local_pos_pub.publish(data)

    def setpoint_sender(self):
        setpoint_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        while(not self.stopped):
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
            if result: # returns True if done
                self.current = self.current + 1
                self.runTask(self)

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
        self.target       = Point(x,y,z)
        self.orientation  = yaw
        self.precision    = Point(precisionXY, precisionXY, precisionZ)
        self.precisionYAW = precisionYAW
        self.sent         = False

    def __str__(self):
        return super(target, self).__str__(self) + "{0} pointing {1} radians".format(self.target, self.orientation)

    def run(self, UAV):
        if(not self.sent):
            UAV.setpoint_position(self.target, yaw):
        return self.isDone(UAV)

    def isDone(self, UAV):
        """ Method to know if the UAV arrived at his position or not

        Better solution for more complex surfaces : 
        http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not/2752753#2752753 """
        if(fabs(UAV.position.x - self.target.x) > self.precision.x):
            return False
        if(fabs(UAV.position.y - self.target.y) > self.precision.y):
            return False
        if(fabs(UAV.position.z - self.target.z) > self.precision.z):
            return False
        if(fabs(UAV.yaw - self.yaw) > self.precisionYAW):
            return False
        # it is done
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
            UAV.setpoint_loiter()

        return self.isDone()

    def isDone(self):
        waitTime.sleep()
        return True

class takeoff(task, object):
    """The takeoff class is a task. It says to the UAV to go to 
    takeoff""" 
    def __init__(self, name, precision=0.05):
        super(takeoff, self).__init__("target", name)
        self.sent             = False
        self.takeoff_altitude = 0.5
        self.precision        = precision

    def __str__(self):
        return super(target, self).__str__(self) + "{0} pointing {1} radians".format(self.target, self.orientation)

    def run(self, UAV):
        if(not self.sent):
            UAV.setpoint_takeoff()
            self.takeoff_altitude = UAV.takeoff_altitude
        return self.isDone(UAV)

    def isDone(self, UAV):
        """ Just verify if we correctly take off or not """
        if(fabs(UAV.position.z - self.takeoff_altitude) > precision):
            return False
        # it is done
        return True


class land(task, object):
    """The land class is a task. It says to the UAV to go to 
    land""" 
    def __init__(self, name, precision=0.05):
        super(land, self).__init__("target", name)
        self.sent             = False
        self.landing_altitude = 0.1
        self.precision        = precision

    def __str__(self):
        return super(target, self).__str__(self) + "{0} pointing {1} radians".format(self.target, self.orientation)

    def run(self, UAV):
        if(not self.sent):
            UAV.setpoint_land()
            self.landing_altitude = UAV.landing_altitude
        return self.isDone(UAV)

    def isDone(self, UAV):
        """ Just verify if we correctly landed or not, maximum landing_altitude + precision (15cm) """
        if(fabs(UAV.position.z - self.landing_altitude) > precision):
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