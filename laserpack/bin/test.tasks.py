from tasks import *
import rospy

rospy.init_node('test_tasks')
Controller = taskController()
task = init_UAV("Init UAV")
Controller.addTask(task)

for i in range(100):
    Controller.rate.sleep()
    Controller.spinOnce()