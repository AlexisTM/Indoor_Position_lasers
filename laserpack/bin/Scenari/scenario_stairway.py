#!/usr/bin/env python
from tasks import *
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

def DoStair(num, size, a=-2, b=2, c=1):
    #StairWay following x axis
    #GO up then GO straight (num times)
    #size give the size of the step
    #Default begin on (-2,2,1)
    x = [a]
    y = [b] * ((num*2)+1)
    z = [c]
    for i in range(num):
        x.extend((x[i*2]       , x[i*2] + size))
        z.extend((z[i*2] + size, z[i*2] + size))
    return [(x[l], y[l], z[l]) for l in range(0,(num*2+1))]

rospy.init_node('test_tasks')
Controller = taskController(rate=3, setpoint_rate=10)
rospy.loginfo("Controller initiated")
signal(SIGINT, signal_handler)


#Initialisation
tasks = []
rospy.loginfo("Stairway sequencing")
tasks.append(init_UAV("Init UAV"))
tasks.append(arm("Arming"))
tasks.append(takeoff("TAKEOFF"))

#Make the stair path
#   Every step, he takes rest
StairPoints = DoStair(3,2)
for i in range(0, len(StairPoints)):
    tasks.append(target("target", Point(StairPoints[i][0], StairPoints[i][1], StairPoints[i][2])))
    if i % 2 == 0:
        tasks.append(loiter("Waiting", WaitingTime))

tasks.append(takeoff("TAKEOFF"))
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
