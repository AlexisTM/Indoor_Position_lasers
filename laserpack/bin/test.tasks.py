from tasks import *
import rospy

rospy.init_node('test_tasks')
Controller = taskController(rate=10, setpoint_rate=10)
rospy.loginfo("Controller initiated")

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

rospy.loginfo("Adding tasks")
task1 = init_UAV("Init UAV")
task2 = target("target1", 10,10,10, 0)
tasks = [task1, task2]
Controller.addTasks(tasks)
rospy.loginfo("Tasks added")


for i in range(3):
    Controller.rate.sleep()
    Controller.spinOnce()
    rospy.loginfo("Controller initiated %s", Controller.getCurrentTask().__str__())

exit()
