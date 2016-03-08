import rospy
import time
from math import fabs
from geometry_msgs.msg import PoseStamped, Quaternion
from transformations import *


class taskController:
    """ The task controller handle a list with every tasks """
    def __init__(self, rate = 10):
        self.tasks = list()
        self.count = 0
        self.current = 0
        self.setRate(rate)
        self.UAV = (0,0,0)
        self.yaw = 0
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.updatePosition)


    def updatePosition(self, topic):
        self.UAV = (topic.pose.position.x, topic.pose.position.y, topic.pose.position.z)
        _, _, yaw = euler_from_quaternion(topic.pose.orientation, axes="sxyz")
        self.yaw = yaw

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
            result = task.run(self.UAV, self.yaw)
            print result
            if result: # returns True if done
                self.current = self.current + 1

        print self.current, self.count
        return 


cl ass task:
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
       
    def run(self, UAV, yaw):
       
        # do something then return isDone
        return self.isDone()

    def getType(self):
        return self.Type

    def __str__(self):
        return "Task {0} - {1}".format(self.type, self.name)



class target(task, object):
    """The target class is a task. It says to the UAV to go to 
    the target""" 
    def __init__(self, name, x, y, z, yaw, precisionXY = 5, precisionZ = 5, precisionYAW = 1):
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


    def run(self, UAV, yaw):
        # Send the position once to the SENDER thread then only check if arrived 
        if not(self.sent) :
            self.sender = rospy.Publisher('/UAV/Setpoint', PoseStamped, queue_size=1)
            msg = SP.PoseStamped()
            msg.pose.position.x = self.target[0]
            msg.pose.position.y = self.target[1]
            msg.pose.position.z = self.target[2]
            msg.pose.orientation = Quaternion(quaternion_from_euler(0, 0, yaw, axes="sxyz"))
            sender.publish(msg)
            self.sent = True
        
        return self.isDone(UAV, yaw)

    def isDone(self, UAV, yaw):
        """ Method to know if the UAV arrived at his position or not

        Better solution for more complex surfaces : 
        http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not/2752753#2752753 """
        if(fabs(UAV[0] - self.target[0]) > self.precisionXY):
            return False
        if(fabs(UAV[1] - self.target[1]) > self.precisionXY):
            return False
        if(fabs(UAV[2] - self.target[2]) > self.precisionZ):
            return False
        if(fabs(yaw - self.yaw) > self.precisionYAW):
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

    def run(self, UAV, yaw):
        return self.isDone()

    def isDone(self):
        """ Method to know if the UAV arrived at his position or not
        http://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not/2752753#2752753 """
        return True

class loiter(task, object):
    """The loiter class is a task. It is just a waiting task"""
    def __init__(self, name, waitTime):
        super(loiter, self).__init__("loiter", name)
        self.waitTime = waitTime
        self.last = None

    def __str__(self):
        return super(loiter, self).__str__()

    def run(self, UAV, yaw):
        if(self.last == None):
            self.last = time.time()
        return self.isDone()

    def isDone(self):
        now = time.time()
        if(now - self.last > self.waitTime):
            return True
        else :
            return False


class test(task, object):
    """The test class is a task. It is just a testing task"""
    def __init__(self, name, waitTime):
        super(test, self).__init__("test", name)
        self.waitTime = waitTime
        self.last = None

    def __str__(self):
        return super(test, self).__str__()

    def run(self, UAV, yaw):
        if(self.last == None):
            self.last = time.time()
            print("first time")
        return self.isDone()

    def isDone(self):
        now = time.time()
        print(now - self.last)
        if(now - self.last > self.waitTime):
            print("done")
            return True
        else :
            print("waiting")
            return False