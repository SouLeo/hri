#!/usr/bin/env python 
import rospy 
from std_msgs.msg import Int32
from drunk_turtle.srv import TurtlePop

class TurtlePopulation(object):
    def __init__(self):
        self.numTurtles = 0
        self.turtlePopPub = rospy.Publisher('turtle_pop', Int32, queue_size=10)

    def updateTurtleNum(self):
        self.numTurtles += 1

    def handle_turtle_pop(self):
        self.turtlePopPub.publish(self.numTurtles)
