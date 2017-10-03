#!/usr/bin/env python 
import rospy
from math import pi, sqrt, atan2
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from DrunkTurtle import DrunkTurtle
from turtlesim.srv import Kill
from turtlesim.msg import Pose

class SocialTurtle(DrunkTurtle):
    def __init__(self, x, y, th):
        # The social turtle will always have an id = 1
        super(SocialTurtle, self).__init__(1, 6, x, y, th)

    def motionPlanner(self):
        # TODO: Override motionPlanner function from DrunkTurtle.py
        # Function: motionPlanner
        # Description: If social robot encounters friends, 
        #    it'll move toward the friends. If it encounters
        #    foes, it will move away.
        # Input: None
        # Output: cmd_vel for Social Turtle
           
        # Make turtle go toward friend and away from foe

        xSo = pedTurtlePoses[0]
        ySo = pedTurtlePoses[1]

        xFr = pedTurtlePoses[2]
        yFr = pedTurtlePoses[3]

        xFo = pedTurtlePoses[4]
        yFo = pedTurtlePoses[5]

 
        if sqrt(pow((xFr - xSo), 2) + pow((yFr - ySo), 2)) <= 15:
            self.velMsg.linear.x = 1.5 * sqrt(pow((xFr - xSo), 2) + pow((yFr - ySo), 2)) - 0.2 * sqrt(pow((xFo - xSo), 2) + pow((yFo - ySo), 2))
            self.velMsg.angular.z = 4 * (atan2(yFr - ySo, xFr - xSo) - self.pose.theta) - 0.1 * (atan2(yFo - ySo, xFo - xSo) - self.pose.theta)
        if sqrt(pow((xFo - xSo), 2) + pow((yFo - ySo), 2)) <= 5:
            self.velMsg.linear.x = 0.2 * sqrt(pow((xFr - xSo), 2) + pow((yFr - ySo), 2)) - 1.5 * sqrt(pow((xFo - xSo), 2) + pow((yFo - ySo), 2))
            self.velMsg.angular.z = 0.1 * (atan2(yFr - ySo, xFr - xSo) - self.pose.theta) - 4 * (atan2(yFo - ySo, xFo - xSo) - self.pose.theta)
        else:
            self.velMsg.linear.x = 1.5 * sqrt(pow((xFr - xSo), 2) + pow((yFr - ySo), 2))
            self.velMsg.angular.z = 4 * (atan2(yFr - ySo, xFr - xSo) - self.pose.theta)

        # update
        self.velocityPublisher.publish(self.velMsg)
        self.rate.sleep()

def getPoses():
    # Function: getPoses
    # Descriptions: gets poses for all pedestrian turtles
    # Input: none
    # Output: Turtle pose
    t1 = rospy.Subscriber("turtle1/pose", Pose, collectPoses1)
    t2 = rospy.Subscriber("turtle2/pose", Pose, collectPoses2)
    t3 = rospy.Subscriber("turtle3/pose", Pose, collectPoses3)

    # TODO: Make this expandable to N pedestrians
    #    numTurtles = msg.data
    #    for i in range(1, numTurtles):
    #        string = "turtle"+str(numTurtles)+"/pose"
    #        rospy.Subscriber(string, Pose, collectPoses)

def collectPoses1(msg):
    pedTurtlePoses[0] = float(msg.x)
    pedTurtlePoses[1] = float(msg.y)
  
    # TODO: Remove Social Turtle from Pedestrian Array.

def collectPoses2(msg):
    # Function: collectPoses
    # Description: stores poses recorded by getPoses
    # Input: pose data
    # Output: Pose Array of pedestrian turtles
    pedTurtlePoses[2] = float(msg.x)
    pedTurtlePoses[3] = float(msg.y)

def collectPoses3(msg):
    pedTurtlePoses[4] = float(msg.x)
    pedTurtlePoses[5] = float(msg.y)


if __name__ == '__main__':
    try:
        rospy.init_node('social_turtle', anonymous = True)
#        turtlePopSub = rospy.Subscriber("turtle_pop", Int32, getPoses) 

        pedTurtlePoses = []
        for i in range(6):
            pedTurtlePoses.append(i)
        rospy.wait_for_service('kill')
        
        # Kill current turtle #1
        killTurtle = rospy.ServiceProxy('kill', Kill)
        killTurtle('turtle1')

        socialTurtle = SocialTurtle(1,3,0)
        while not rospy.is_shutdown():
            getPoses()
            socialTurtle.motionPlanner()
    except rospy.ROSInterruptException: pass
