from tasks import *
import rospy

rospy.init_node('test_tasks')
Controller = taskController( rate=3)

tasks = []

"""
# Typical fill
for data in dataIn :
    if data.type == "init" :
        task = init_UAV("Init UAV")
    if data.type == "target" :
        task = target("target1", 10,10,10, 0)

    tasks.append(task)
"""

task1 = init_UAV("Init UAV")
task2 = target("target1", 10,10,10, 0)

tasks = [task1, task2]
Controller.addTasks(tasks)


for i in range(3):
    Controller.rate.sleep()
    Controller.spinOnce()