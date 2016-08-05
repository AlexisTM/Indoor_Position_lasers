#!/usr/bin/env python
from tasks import *
import rospy
from signal import signal, SIGINT
from geometry_msgs.msg import Point
import sys

def signal_handler(signal, frame):
        print('You pressed Ctrl+C')
        print('Leaving the Controller & closing the UAV')
        Controller.__exit__()
        sys.exit(0)

rospy.init_node('test_tasks')
Controller = taskController(rate=3, setpoint_rate=10)
rospy.loginfo("Controller initiated")
signal(SIGINT, signal_handler)


tasks = []

"""
# Typical way to fill from any source
for data in dataIn :
    if data.type == "init" :
        task = init_UAV("Init UAV")
    if data.type == "target" :
        task = target("target1", 10,10,10, 0)

    tasks.append(task)
"""

rospy.loginfo("Adding tasks")
task1 = init_UAV("Init UAV")

# target("name", destination, yaw, precision_xy, precision_z, precision yaw)
# destination = Point(0,0,0)
# yaw = 0 - default
# precision_xy = 0.05 - default
# precision_z = 0.05 - default
# precision_yaw = 1 - default

task2 = target("target1", Point(0,0,1))
task3 = arm("Arming")
task4 = disarm("Disarming")
tasks = [task1, task3, task2, task4]
Controller.addTasks(tasks)
rospy.loginfo("Tasks added")


# for i in range(100):
while True:
    Controller.rate.sleep()
    Controller.spinOnce()
    rospy.loginfo("Task %s on %s : %s", Controller.current+1, Controller.count, Controller.getCurrentTask().__str__())

Controller.__exit__()
sys.exit(0)
