#!/usr/bin/env python
import rospy
from DrunkTurtle import DrunkTurtle

class PedestrianTurtle(DrunkTurtle):
    def __init__(self, turtleNum, pattern, isFriend, radius):
        super(PedestrianTurtle, self).__init__(turtleNum, pattern)  
        self.isFriend = isFriend
        self.radius = radius

#    def personalSpaceCost(self):
        # Function: personalSpaceCost
        # Description: ascribes concentric regions around a turtle cost values
        # Input: none, but uses turtleID
        # Output: Sets values surrounding turtle (to prevent other turtles from
        # encroaching on its personal space)

#    def motionPlanner(self, isMoving):
        # Function: motionPlanner
        # Description: contains an algorithm for pedestrian turtles
        # Input: Boolean to determine if pedestrian will be static or dynamic
        # Output: 
        #   1) If static, the turtle will remain in its spawn location
        #   2) If dynamic, the turtle will move in a confined area
 #       if (isMoving == True):
            # set a bounding box for turtle to randomly move in
#        else (isMoving == False):
            # set spawn location for turtle

if __name__ == '__main__':
    try: 
        rospy.init_node('pedestrian_turtles', anonymous = True)
        friend = PedestrianTurtle(1,4,True,3)
        foe = PedestrianTurtle(1,4,False,5)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException: pass
