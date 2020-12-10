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
from asa_ros_msgs.srv import FindAnchor

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
        rospy.loginfo(self.SpatialAnchors[0])
        anchor_ids = [x["Id"] for x in self.SpatialAnchors]
        return anchor_ids

class ASAHandler:
    def __init__(self):
        rospy.wait_for_service('asa_ros/find_anchor')
        self.find_anchor_service = rospy.ServiceProxy('asa_ros/find_anchor', FindAnchor)


    def find_anchor(self, anchor_id):
        rospy.loginfo("Trying to find anchor with id " + anchor_id)
        response = self.find_anchor_service(anchor_id)
        rospy.loginfo(response)

    def find_anchors(self, anchor_ids):
        for id in anchor_ids:
            self.find_anchor(id)


class MissionExecuter:


    def __init__(self):

        rospy.init_node('mission_executer', anonymous=True) 

        self.mission = None
        self.asa_handler = ASAHandler()

        jsonString = '{"$id":"1","SpatialAnchors":[{"$id":"4","IsRoot":true,"Id":"2da468a2-cf0a-4435-9379-cd4016659061","RelativePoseToParent":{"$id":"5","X":0,"Y":0,"Z":0,"i":0,"j":0,"k":0,"w":1},"ParentAnchorId":"root"}],"Waypoints":[{"$id":"6","$type":"WaypointControl.Waypoint.Waypoint, WaypointControl","Name":"EntryPoint","Anchor":{"$ref":"4"},"Pose":{"$id":"7","$type":"RobotUtilities.Pose, RobotUtilities","X":0,"Y":0,"Z":0,"i":0,"j":0,"k":0,"w":1},"LifetimeId":"WP1","TargetSetTasks":[],"EntryTasks":[],"MainTasks":[],"ExitTasks":[]},{"$id":"8","$type":"WaypointControl.Waypoint.Waypoint, WaypointControl","Name":"wp1","Anchor":{"$ref":"4"},"Pose":{"$id":"9","$type":"RobotUtilities.Pose, RobotUtilities","X":1.5,"Y":0,"Z":0,"i":0,"j":0,"k":0,"w":1},"LifetimeId":"WP2","TargetSetTasks":[],"EntryTasks":[],"MainTasks":[],"ExitTasks":[]}],"Edges":[{"$id":"10","$type":"WaypointControl.Edge.Edge, WaypointControl","Source":{"$ref":"6"},"Target":{"$ref":"8"}}],"EntryPoint":{"$ref":"6"}}'

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
