#!/usr/bin/env python
import rospy
import random
from math import pi, sqrt, atan2
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

class DrunkTurtle:
    def __init__(self, turtleNum, pattern):
        self.motionPath = pattern
        self.turtleID = turtleNum
        self.isTeleop = False
        self.distanceTolerance = 2
        
        # ROS Object Construction 
        self.velMsg = Twist()
        self.pose = Pose()
        self.goalPose = Pose()

        # ROS Topic Publishers and Subscribers
        self.velocityTopic = '/turtle' + str(self.turtleID) + '/cmd_vel'
        self.poseTopic = '/turtle' + str(self.turtleID) + '/pose'
        self.velocityPublisher = rospy.Publisher(self.velocityTopic, Twist, queue_size = 10)
        self.poseSubscriber = rospy.Subscriber(self.poseTopic, Pose, self.poseCallback)
        self.posePublisher = rospy.Publisher(self.poseTopic, Pose, queue_size = 10) 
        self.rate = rospy.Rate(10)

    def poseCallback(self, data):
    # Function: poseCallback
    # Description: updates the actual pose of the turtlesim
    # Input: current pose
    # Output: updates current pose for turtle
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def getDistanceError(self):
    # Function: getDistanceError
    # Description: determines how far our turtle is from its goal pose
    # Input: none, but gets currentPose data from turtle's member variables
    # Output: returns the distance between the turtle and the goal pose
        distance = sqrt(pow((self.goalPose.x - self.pose.x), 2) + pow((self.goalPose.y - self.pose.y), 2)) 
        return distance

    def setPose(self,x,y,theta): 
    # Function: setPose
    # Description: sets the turtlesim's pose
    # Input: x,y,theta
    # Output: respawn turtlesim at location
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta
        rospy.wait_for_service('turtle' + str(self.turtleID) + '/teleport_absolute')
        teleportTurtle = rospy.ServiceProxy('turtle' + str(self.turtleID) + '/teleport_absolute', TeleportAbsolute)
        teleportTurtle(x, y, theta)

    def setGoalPose(self,x,y,theta):
    # Function: setGoalPose
    # Description: sets the turtlesim's goal pose
    # Output: sets the turtlesim's end goal location
        self.goalPose.x = x
        self.goalPose.y = y
        self.pose.theta = theta

    def motionPlanner(self):
    # Function: motionPlanner
    # Description:  contains algorithms for navigating in
    #      1. straight line
    #      2. circle
    #      3. lawn mower pattern
    #      4. go to goal
    #      5. random
    # Input: pattern type and turtle number
    # Output: cmd_vel for given turtle
        if (self.motionPath == 0):
        # Turtlesim moves in straight line
            self.velMsg.linear.x = 1.0
            self.velMsg.linear.y = 0.0
            self.velMsg.linear.z = 0.0
            self.velMsg.angular.x = 0.0
            self.velMsg.angular.y = 0.0
            self.velMsg.angular.z = 0.0
        
        elif (self.motionPath == 1):
        # Turtlesim moves in circle
            self.velMsg.linear.x = 10.0
            self.velMsg.linear.y = -10.0
            self.velMsg.linear.z = 0.0
            self.velMsg.angular.x = 0.0
            self.velMsg.angular.y = 0.0
            self.velMsg.angular.z = 10.0
        
        elif (self.motionPath == 3): 
        # Turtlesim moves toward a goal pose
            while sqrt(pow((self.goalPose.x - self.pose.x), 2) + pow((self.goalPose.y - self.pose.y), 2)) >= self.distanceTolerance:
                # linear correction along x axis
                self.velMsg.linear.x = 1.5 * sqrt(pow((self.goalPose.x - self.pose.x), 2) + pow((self.goalPose.y - self.pose.y), 2))
                self.velMsg.linear.y = 0
                self.velMsg.linear.z = 0
                # angular correction around z axis
                self.velMsg.angular.x = 0 
                self.velMsg.angular.y = 0
                self.velMsg.angular.z = 4 * (atan2(self.goalPose.y - self.pose.y, self.goalPose.x - self.pose.x) - self.pose.theta)
                # update
                self.velocityPublisher.publish(self.velMsg)
                self.rate.sleep()
            self.velMsg.linear.x = 0
            self.velMsg.angular.z = 0
            self.velocityPublisher.publish(self.velMsg)

        elif (self.motionPath == 4):
        # Turtlesim moves in a random pattern
            self.velMsg.linear.x = random.random() * 2
            self.velMsg.linear.y = random.random() * 2
            self.velMsg.linear.z = random.random() * 2
            self.velMsg.angular.x = random.random() * 2 * pi 
            self.velMsg.angular.y = random.random() * 2 * pi
            if (random.random() < 0.5):
                self.velMsg.angular.z = random.random() * 5 * pi
            else:
                self.velMsg.angular.z = random.random() * -5 * pi    

if __name__ == '__main__':
    try: 
        rospy.init_node('drunken_turtles', anonymous = True)
        drunkTurtle = DrunkTurtle(1,3)
        drunkTurtle.setGoalPose(5,9,3)
        drunkTurtle.setPose(20,5,5)
        while not rospy.is_shutdown():
            drunkTurtle.motionPlanner()
            drunkTurtle.velocityPublisher.publish(drunkTurtle.velMsg)
    except rospy.ROSInterruptException: pass
