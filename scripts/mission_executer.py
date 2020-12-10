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

import json
import pprint

class Mission(object):
    def __init__(self, jsonString):
        self.__dict__.update(json.loads(jsonString))

    def get_anchor_ids(self):
        anchor_ids = [x.Id for x in self.SpatialAnchors]
        return anchor_ids

class ASAHandler:
    def __init__(self):
        rospy.wait_for_service('asa_ros/find_anchor')
        self.find_anchor_service = rospy.ServiceProxy('asa_ros/find_anchor', String)


    def find_anchor(self, anchor_id):
        rospy.loginfo("Trying to find anchor with id " + anchor_id)
        msg = "anchor_id: '" + anchor_id + "'"
        response = self.trajectory_service(msg)
        rospy.loginfo(response)

    def find_anchors(self, anchor_ids):
        for id in anchor_ids:
            self.find_anchor(id)


class MissionExecuter:


    def __init__(self):

        rospy.init_node('mission_executer', anonymous=True) 

        self.mission = None
        self.asa_handler = ASAHandler()

        jsonString = '{"$id":"1","SpatialAnchors":[{"$id":"2","IsRoot":false,"Id":"id","RelativePoseToParent":{"$id":"3","X":0.0,"Y":0.0,"Z":0.0,"i":0.0,"j":0.0,"k":0.0,"w":1.0},"ParentAnchorId":"root"},{"$id":"4","IsRoot":false,"Id":"id","RelativePoseToParent":{"$id":"5","X":0.0,"Y":0.0,"Z":0.0,"i":0.0,"j":0.0,"k":0.0,"w":1.0},"ParentAnchorId":"root"}],"Waypoints":[{"$id":"6","$type":"WaypointControl.Waypoint.Waypoint, WaypointControl","Name":"EntryPoint","Anchor":{"$ref":"2"},"Pose":{"$id":"7","$type":"RobotUtilities.Pose, RobotUtilities","X":0.0,"Y":0.0,"Z":0.0,"i":0.0,"j":0.0,"k":0.0,"w":1.0},"LifetimeId":"bf5afe04-31b8-438e-9f9f-7c040b52bb48","TargetSetTasks":[],"EntryTasks":[],"MainTasks":[],"ExitTasks":[]},{"$id":"8","$type":"WaypointControl.Waypoint.Waypoint, WaypointControl","Name":"wp1","Anchor":{"$ref":"4"},"Pose":{"$id":"9","$type":"RobotUtilities.Pose, RobotUtilities","X":0.0,"Y":0.0,"Z":0.0,"i":0.0,"j":0.0,"k":0.0,"w":1.0},"LifetimeId":"2fd81f40-0157-4d45-88ee-8dd43ea9374d","TargetSetTasks":[],"EntryTasks":[],"MainTasks":[],"ExitTasks":[]}],"Edges":[{"$id":"10","$type":"WaypointControl.Edge.Edge, WaypointControl","Source":{"$ref":"6"},"Target":{"$ref":"8"}}],"EntryPoint":{"$ref":"6"}}'

        self.load_mission(jsonString)

        self.asa_handler.find_anchors(self.mission.get_anchor_ids())

        rospy.loginfo("Initialized mission_executer")       

    def spin(self):
        rospy.spin()

###################
# Mission loading #
###################

    def load_mission(self, jsonString):
        self.mission = Mission(jsonString)


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
