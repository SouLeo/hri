#!/usr/bin/env python
import rospy
import random
from math import pi
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class DrunkTurtle:
    def __init__(self, turtleNum, pattern):
        self.motionPath = pattern
        self.turtleID = turtleNum
        self.velocityTopic = '/turtle' + str(self.turtleID) + '/cmd_vel'
        self.poseTopic = '/turtle' + str(self.turtleID) + '/pose'
        self.velocityPublisher = rospy.Publisher(self.velocityTopic, Twist, queue_size = 10)
        self.poseSubscriber = rosy.Subscriber(self.poseTopic, Pose, self.poseCallback)
        self.posePublisher = rospy.Publisher(self.poseTopic, Pose, queue_size = 10)
        self.velMsg = Twist()
        self.pose = Pose()
        self.goalPose = Pose()
        self.rate = rospyRate(10)
        self.isTeleop = False
        self.distanceTolerance = 2

    def poseCallback(self, data):
    # Function: poseCallback
    # Description: Updates the actual pose of the turtlesim
    # Input: Current Pose
    # Output: Updates Current Pose for turtle
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def getDistanceError(self):
        distance = sqrt(pow((self.goalPose.x - self.pose.x), 2) + pow((self.goalPose.y - self.pose.y), 2)) 
        return distance

    def setPose(self,x,y,theta): #TODO: TEST THAT PUBLISHER WORKS 
    # Function: setPose
    # Description: Sets the turtlesim's pose
    # Input: x,y,theta
    # Output: respawn turtlesim at location
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta
        self.posePublisher.publish(self.pose)

    def setGoalPose(self,x,y,theta):
    # Function: setGoalPose
    # Description: Sets the turtlesim's goal pose
    # Output: sets the turtlesim's end goal location
        self.goalPose.x = x
        self.goalPose.y = y
        self.pose.theta = theta

    def motionPlanner(self):
    # Function: motionPlanner
    # Description:  Contains algorithms for navigating in
    #      1. Straight Line
    #      2. Circle
    #      3. Lawn Mower Pattern
    #      4. Going to Point
    #      5. Random
    # Input: Pattern Type and turtle number
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
        
        elif (self.motionPath == 3): # TODO: Test and verify this works
        # Turtlesim moves toward a goal pose
            while sqrt(pow((self.goalPose.x - self.pose.x), 2) + pow((self.goalPose.y - self.pose.y), 2)) >= self.distanceTolerance:
                # linear correction along x axis
                self.velMsg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
                self.velMsg.linear.y = 0
                self.velMsg.linear.z = 0
                
                # angular correction around z axis
                self.velMsg.angular.x = 0 
                self.velMsg.angular.y = 0
                self.velMsg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
               
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
        drunkTurtle = DrunkTurtle(1,4)
        drunkTurtle.setPose(40,20,3)
        while not rospy.is_shutdown():
            drunkTurtle.motionPlanner()
            drunkTurtle.velocityPublisher.publish(drunkTurtle.velMsg)
    except rospy.ROSInterruptException: pass
