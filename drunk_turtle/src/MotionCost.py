#!/usr/bin/env python 
import rospy from DrunkTurtle 
import DrunkTurtle 

class MotionCost(DrunkTurtle): def __init__(self, turtleNum, pattern):
    self.poseArraySubscriber = rospy.Subscriber(self.poseArrayTopic, PoseArray, self.poseArrayCallback)

    def poseArrayCallback(self, data):
        # create pose array global variable that each individual turtle will
        # update its own pose. 

    def motionPlanner(self):
        # Function: motionPlanner Description: contains an algorithm for active
        # turtle to reach goal location Input: Array of current drunk turtle
        # poses Output: cmd_vel for active turtle to reach goal

