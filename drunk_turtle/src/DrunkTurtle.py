#!/usr/bin/env python
import rospy
import random
from math import pi, sqrt, atan2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn
from turtlesim.srv import Kill

class DrunkTurtle(object):
    def __init__(self, turtleNum, pattern, x, y, th):
        self.motionPath = pattern
        self.turtleID = turtleNum
        self.isTeleop = False
        self.distanceTolerance = 2
        
        spawnTurtle = rospy.ServiceProxy('spawn', Spawn)
        spawnTurtle(x, y, th, 'turtle' + str(self.turtleID))
        
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
        
        # Turtle Motion Begins
        self.motionPlanner()

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
    #      6. stationary
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
            self.velocityPublisher.publish(self.velMsg)
        
        elif (self.motionPath == 1):
        # Turtlesim moves in circle
            self.velMsg.linear.x = 10.0
            self.velMsg.linear.y = -10.0
            self.velMsg.linear.z = 0.0
            self.velMsg.angular.x = 0.0
            self.velMsg.angular.y = 0.0
            self.velMsg.angular.z = 10.0
            self.velocityPublisher.publish(self.velMsg)
        
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
            self.velocityPublisher.publish(self.velMsg)

        elif (self.motionPath == 5): 
            self.velMsg.linear.x = 0 
            self.velMsg.linear.y = 0
            self.velMsg.linear.z = 0
            self.velMsg.angular.x = 0 
            self.velMsg.angular.y = 0
            self.velMsg.angular.z = 0
            self.velocityPublisher.publish(self.velMsg)
        
        elif (self.motionPath == 6): 
            pass 

def callback(data):
    velocity = Twist()
    if "forward" in data.data:
        rospy.loginfo("forward")
        velocity.linear.x = 1.0
        pub.publish(velocity)
    elif "backward" in data.data:
        rospy.loginfo("backward")
        velocity.linear.x = -1.0
        pub.publish(velocity)
    else:
        rospy.loginfo("nada heard")
        rospy.loginfo("%s", data.data)
        velocity.linear.x = 0.0
        pub.publish(velocity)

if __name__ == '__main__':
    try: 
        rospy.init_node('drunken_turtles', anonymous = True)
        
        # Kill the starting turtle
        rospy.wait_for_service('kill')
        killTurtle = rospy.ServiceProxy('kill', Kill)
        killTurtle('turtle1')

        # Subscribe to /pocketsphinx_recognizer/output
        psSub = rospy.Subscriber('/pocketsphinx_recognizer/output', String, callback)
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)


        drunkTurtle = DrunkTurtle(1,6,5,5,5)
        while not rospy.is_shutdown():
            drunkTurtle.motionPlanner()
    except rospy.ROSInterruptException: pass
