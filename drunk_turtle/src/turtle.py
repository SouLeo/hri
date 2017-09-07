#!/usr/bin/env python
import rospy
import random
from math import pi
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Turtle:
    def __init__(self, turtleNum, pattern):
        self.motionPath = pattern
        self.turtleID = turtleNum
        self.velocityTopic = '/turtle' + str(self.turtleID) + '/cmd_vel'
        self.velocityPublisher = rospy.Publisher(self.velocityTopic, Twist, queue_size = 10)
        self.velMsg = Twist()
        self.pose = Pose()
        self.isTeleop = False

    # Function: setPose
    # Description: Sets the turtlesim's pose
    # Input: x,y,theta
    # Output: respawn turtlesim at location
    def setPose(self,x,y,theta):
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta

    # Function: motionPlanner
    # Description:  Contains algorithms for navigating in
    #       1. Straight Line
    #       2. Circle
    #       3. Lawn Mower Pattern
    #       4. Going to Point
    #       5. Random
    # Input: Pattern Type and turtle number
    # Output: cmd_vel for given turtle
    def motionPlanner(self):
        if (self.turtleID == 0):
            self.velMsg.linear.x = 1.0
            self.velMsg.linear.y = 0.0
            self.velMsg.linear.z = 0.0
            self.velMsg.angular.x = 0.0
            self.velMsg.angular.y = 0.0
            self.velMsg.angular.z = 0.0
        elif (self.turtleID == 1):
            self.velMsg.linear.x = 10.0
            self.velMsg.linear.y = -10.0
            self.velMsg.linear.z = 0.0
            self.velMsg.angular.x = 0.0
            self.velMsg.angular.y = 0.0
            self.velMsg.angular.z = 10.0
        elif (self.turtleID == 4):    
            self.velMsg.linear.x = random.random() * 20
            self.velMsg.linear.y = random.random() * 20
            self.velMsg.linear.z = random.random() * 20
            self.velMsg.angular.x = random.random() * 2 * pi 
            self.velMsg.angular.y = random.random() * 2 * pi
            self.velMsg.angular.z = random.random() * 20 * pi

if __name__ == '__main__':
    try: 
        rospy.init_node('drunken_turtles', anonymous = True)
        drunkTurtle = Turtle(1,1)
        drunkTurtle.setPose(40,20,3)
        while not rospy.is_shutdown():
            drunkTurtle.motionPlanner()
            drunkTurtle.velocityPublisher.publish(drunkTurtle.velMsg)
    except rospy.ROSInterruptException: pass
