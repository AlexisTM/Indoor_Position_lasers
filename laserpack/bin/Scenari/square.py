#!/usr/bin/env python
from tasks import *
from math import cos, sin, pi
import rospy
from signal import signal, SIGINT
from geometry_msgs.msg import Point
import sys
WaitingTime = 10 #seconds

def signal_handler(signal, frame):
        print('You pressed Ctrl+C')
        print('Leaving the Controller & closing the UAV')
        Controller.__exit__()
        sys.exit(0)

def PointsInCircum(r,z=3,n=4):
    #return (x,y,z) points of a square
    #PointsInCircum(Rayon(m), Altitude{defaut:3}(m), NombreDePoints{defaut:4})
    return [(round(cos(2*pi/n*x)*r,3),round(sin(2*pi/n*x)*r,3),z) for x in range(0,n+1)]

rospy.init_node('test_tasks')
Controller = taskController(rate=3, setpoint_rate=10)
rospy.loginfo("Controller initiated")
signal(SIGINT, signal_handler)


#Initialisation
tasks = []
rospy.loginfo("Square sequencing")
tasks.append(init_UAV("Init UAV"))
tasks.append(arm("Arming"))

#Targetting Square and wait for seconds
CirclePoints = PointsInCircum(3)
for circle in CirclePoints:
    tasks.append(target("target", Point(circle)))
    tasks.append(loiter("Waiting", WaitingTime))

#Disarming
tasks.append(disarm("Disarming"))

#Adding tasks
Controller.addTasks(tasks)
rospy.loginfo("Tasks added")

# for i in range(100):
while True:
    Controller.rate.sleep()
    Controller.spinOnce()
    rospy.loginfo("Task %s on %s : %s", Controller.current+1, Controller.count, Controller.getCurrentTask().__str__())

Controller.__exit__()
sys.exit(0)
