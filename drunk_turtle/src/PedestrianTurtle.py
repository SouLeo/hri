#!/usr/bin/env python
import rospy
from DrunkTurtle import DrunkTurtle
from TurtlePopulation import TurtlePopulation

class PedestrianTurtle(DrunkTurtle):

    def __init__(self, turtleNum, pattern, x, y, th, isFriend, radius):
        super(PedestrianTurtle, self).__init__(turtleNum, pattern, x, y, th)  
        self.isFriend = isFriend
        self.radius = radius
        turtlePop.updateTurtleNum()

#    def personalSpaceCost(self):
        # Function: personalSpaceCost
        # Description: ascribes concentric regions around a turtle cost values
        # Input: none, but uses turtleID
        # Output: Sets values surrounding turtle (to prevent other turtles from
        # encroaching on its personal space)

if __name__ == '__main__':
    try: 
        rospy.init_node('pedestrian_turtles', anonymous = True)
        turtlePop = TurtlePopulation()
        friend = PedestrianTurtle(2, 4, 5, 5, 0, True, 3)
        foe = PedestrianTurtle(3, 4, 8, 8, 0, False, 5)
        while not rospy.is_shutdown():
            friend.motionPlanner()
            foe.motionPlanner()
            turtlePop.handle_turtle_pop()
    except rospy.ROSInterruptException: pass
