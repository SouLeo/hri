#!/usr/bin/env 
import rospy from DrunkTurtle 
import DrunkTurtle 
from turtlesim.srv import Kill

class SocialTurtle(DrunkTurtle):
    def __init__(self, x, y, th):
        super(SocialTurtle, self).__init__(1, 0, x, y, th)
        # The social turtle will always have an id = 1

#    def getPoses(self):
       # 1. For all turtles (int i = 1, i < numTurtles, i++) 
       #    get poses into an array
       #
       # 2. Once poses are recorded, signal motionPlanner to run (cb)

#   def motionPlanner(self):
       # Function: motionPlanner
       # Description: If social robot encounters friends, 
       #    it'll move toward the friends. If it encounters
       #    foes, it will move away.
       # Input: None
       # Output: cmd_vel for Social Turtle
#       if not self.motionPath:
           # Should always go into this conditional. 
           # Get list of all friend turtle poses. 
           # Get list of all foe turtle poses.
           # Make turtle go toward friend and away from foe
           # Add up cmd_vel correction
           # Publish new cmd_vel

    if __name__ == '__main__':
        rospy.init_node('social_turtle', anonymous = True)
        # Kill current turtle #1
        killTurtle = rospy.ServiceProxy('kill', Kill)
        killTurtle('turtle1')
        socialTurtle = SocialTurtle(3,3,0)
        while not rospy.is_shutdown():
            socialTurtle.motionPlanner()
        except rospy.ROSSerializationException()
