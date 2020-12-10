#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
import tf2_ros as tf
from tf.transformations import *
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from asa_ros_commander.msg import AsaRelPoseStamped

#for mocking
#from asa_ros_commander.msg import CreatedAnchorMock as CreatedAnchor
#from asa_ros_commander.msg import FoundAnchorMock as FoundAnchor
#for real stuff
from asa_ros_msgs.msg import CreatedAnchor
from asa_ros_msgs.msg import FoundAnchor

import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from spot_msgs.srv import Trajectory, TrajectoryResponse
import actionlib
from actionlib import GoalID



class MissionExecuter:


    def __init__(self):

        rospy.init_node('mission_executer', anonymous=True)
        

        rospy.loginfo("Initialized mission_executer")       

    def spin(self):
        rospy.spin()




#########
# Utils #
#########

    

#####################
# Anchor management #
#####################

    

################
# Republishers #
################



if __name__ == '__main__':
    executer = MissionExecuter()
    executer.spin()
