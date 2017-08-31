#!/usr/bin/env python
import rospy
import random
from math import pi
from geometry_msgs.msg import Twist

class Turtle:
    def __init__(self):
        self.velocityPublisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        self.velMsg = Twist()
        self.isTeleop = True
    
    def updateVelocity(self):
        self.velMsg.linear.x = random.random() * 20
        self.velMsg.linear.y = random.random() * 20
        self.velMsg.linear.z = random.random() * 20
        self.velMsg.angular.x = random.random() * 2 * pi 
        self.velMsg.angular.y = random.random() * 2 * pi
        self.velMsg.angular.z = random.random() * 2 * pi

if __name__ == '__main__':
    try: 
        rospy.init_node('drunken_turtles', anonymous = True)
        drunkTurtle = Turtle()
        while not rospy.is_shutdown():
            drunkTurtle.updateVelocity()
            drunkTurtle.velocityPublisher.publish(drunkTurtle.velMsg)
    except rospy.ROSInterruptException: pass
