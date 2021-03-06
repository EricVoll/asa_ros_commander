#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
import tf2_ros as tf
from tf.transformations import *
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped, Pose
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

import json
import pprint

import sys

from azure.cosmos import exceptions, CosmosClient, PartitionKey

class Mission(object):
    def __init__(self, jsonString):
        self.__dict__.update(json.loads(jsonString))

    def get_anchor_ids(self):
        anchor_ids = [x["Id"] for x in self.SpatialAnchors]
        return anchor_ids

class ASAHandler:
    def __init__(self):
        #fields
        self.anchor_statusses = {}
        self.all_anchors_found_callback = None
        self.current_anchor_find_index = 0
        self.anchor_id_list = []

        #publishers, subscribers and services
        rospy.loginfo("Waiting for service: asa_ros/find_anchor")
        rospy.wait_for_service('asa_ros/find_anchor')
        self.find_anchor_service = rospy.ServiceProxy('asa_ros/find_anchor', FindAnchor)
        rospy.Subscriber('/asa_ros/found_anchor', FoundAnchor, self.asa_found_anchor_callback)


    def find_anchor(self, anchor_id):
        rospy.loginfo("Trying to find anchor with id " + anchor_id)
        response = self.find_anchor_service(anchor_id)
        rospy.loginfo(response)

    def find_anchors(self, anchor_ids, found_all_callback):
        self.all_anchors_found_callback = found_all_callback
        for id in anchor_ids:
            self.anchor_statusses[id] = False
            #self.find_anchor(id)

        #find first anchor
        self.anchor_id_list = anchor_ids
        self.find_anchor(self.anchor_id_list[0])

    def asa_found_anchor_callback(self, anchor):
        #Mark anchor as found
        self.anchor_statusses[anchor.anchor_id] = True

        #Check if we found all requested anchors
        all_anchors_found = all(x == True for x in self.anchor_statusses.values())

        if(all_anchors_found):
            self.all_anchors_found_callback()
        else:
            rospy.loginfo("Will continue to look for other anchors:")
            rospy.loginfo(self.anchor_statusses)

            self.current_anchor_find_index += 1
            self.find_anchor(self.anchor_id_list[self.current_anchor_find_index])
        



class MissionExecuter:
    def __init__(self):
        rospy.init_node('mission_executer', anonymous=True) 

        myargv = rospy.myargv(argv = sys.argv)
        if(len(myargv) < 6):
            rospy.logerr("Missing argument for the mission_executer! Usage: mission_executer.py 'mission_file_path_or_id:string' 'orientation_check_enabled:bool' 'tolerance_rotation:float' 'tolerance_translation:float' 'load_mission_from_cosmos_db:bool'")

        self.options = {
            "mission_file_path_or_id" : myargv[1],
            "orientation_check_enabled" : myargv[2] == 'True',
            "tolerance_rotation" : float(myargv[3]),
            "tolerance_translation" : float(myargv[4]),
            "load_mission_from_cosmos_db" : myargv[5] == 'True'
        }

        if(self.options["load_mission_from_cosmos_db"]):
            if(len(myargv) < 7):
                rospy.logerr("Missing argument for the mission_executer! The last argument must be set if comsos db is used. Usage: mission_executer.py 'mission_file_path_or_id:string' 'orientation_check_enabled:bool' 'tolerance_rotation:float' 'tolerance_translation:float' 'load_mission_from_cosmos_db:bool'  'cosmos_db_master_key_file:string' ")

            self.options["comsos_db_master_key_file"] = myargv[6]


        # general fields and objects
        self.mission = None
        self.asa_handler = ASAHandler()

        # state machine fields
        self.current_waypoint = None
        self.current_goal_pose = None
        self.current_anchor_id = None
        self.orientation_check_enabled = self.options["orientation_check_enabled"]

        # ros publishers, subscribers and services
        self.goal_publisher = rospy.Publisher('/anchored_goal', AsaRelPoseStamped, queue_size=10)
        rospy.Subscriber('/odometry/filtered/asa_relative', Odometry, self.robot_position_updated)
        
        # TF objects
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)

        # Mission importer

        if(self.options["load_mission_from_cosmos_db"]):
            masterKey = ""
            with open(self.options["comsos_db_master_key_file"], 'r') as file:
                masterKey = file.read()

            self.load_mission_from_cosmos_db(
                "https://mr-spot-control-mission-db-v2.documents.azure.com:443",
                masterKey,
                self.options["mission_file_path_or_id"]
            )
        else:
            mission_file_path = self.options["mission_file_path_or_id"]
            rospy.loginfo("Loading mission " + mission_file_path)
            with open(mission_file_path, 'r') as file:
                jsonString = file.read().replace('\n', '')
            self.load_mission(jsonString)

        # Kick off anchor finding
        self.asa_handler.find_anchors(self.mission.get_anchor_ids(), self.all_anchors_found_callback)

        rospy.loginfo("Initialized mission_executer")       

    def spin(self):
        rospy.spin()



###################
# Mission loading #
###################

    def load_mission(self, jsonString):
        self.mission = Mission(jsonString)

        #fix anchor assignments
        for waypoint in self.mission.Waypoints:
            anchor = self.find_object_with_id(self.mission.SpatialAnchors, waypoint["Anchor"])
            if(anchor == None):
                rospy.logerror("Could not find the anchor_id of waypoint " + waypoint["LifetimeId"])
            else:
                waypoint["anchor_id"] = anchor["Id"]

    def load_mission_from_cosmos_db(self, endpoint, masterkey, mission_id):
        client = CosmosClient(endpoint, masterkey)
        database_name = 'missions'
        database = client.create_database_if_not_exists(id=database_name)
        container_name = 'stored'
        container = database.create_container_if_not_exists(
            id = container_name,
            partition_key = PartitionKey(path="/fakePartitionKey"),
            offer_throughput=400
        )

        item_response = container.read_item(item=mission_id, partition_key="fakePartitionKey")
        request_charge = container.client_connection.last_response_headers['x-ms-request-charge']
        print('COSMOS DB: item fetched with id {0}. Operation consumed {1} request units'.format(item_response['id'], (request_charge)))

        self.load_mission(item_response["json"])

#########
# Utils #
#########

    def unity2ros_position(self, position):
        return [position[2], -position[0], position[1]]

    def unity2ros_quaternion(self, quaternion):
        return [-quaternion[2], quaternion[0], -quaternion[1], quaternion[3]]

    # Takes a waypoint and publishes the corresponding goal to the correct topic to command the robot.
    def command_robot_to(self, waypoint):

        rospy.loginfo(waypoint)

        # convert unity coordintaes to ros coordinates (left handed to right handed)
        ros_pos = self.unity2ros_position([waypoint["Pose"]["X"], waypoint["Pose"]["Y"], waypoint["Pose"]["Z"]])
        ros_rot = self.unity2ros_quaternion([waypoint["Pose"]["i"], waypoint["Pose"]["j"], waypoint["Pose"]["k"], waypoint["Pose"]["w"]])

        target = AsaRelPoseStamped()
        target.pose = Pose()
        target.pose.position.x =    ros_pos[0]
        target.pose.position.y =    ros_pos[1]
        target.pose.position.z =    ros_pos[2]
        target.pose.orientation.x = ros_rot[0]
        target.pose.orientation.y = ros_rot[1]
        target.pose.orientation.z = ros_rot[2]
        target.pose.orientation.w = ros_rot[3]
        target.anchor_id = waypoint["anchor_id"]

        self.current_goal_pose = target.pose
        self.current_anchor_id = waypoint["anchor_id"]

        self.goal_publisher.publish(target)

    # Finds the object in the list which has an $id matching the $ref field of the object_ref object
    def find_object_with_id(self, list, object_ref):
        return next((x for x in list if x["$id"] == object_ref["$ref"]), None)


#####################
# Anchor management #
#####################

    def all_anchors_found_callback(self):
        rospy.loginfo("found all anchors! We can start")
        
        self.perform_step()
    

#################
# State machine #
#################

    def perform_step(self):
        #Find the next or first target
        if(self.current_waypoint == None):
            self.current_waypoint = self.find_object_with_id(self.mission.Waypoints, self.mission.EntryPoint)
        else:
            #find the next waypoint according to the edges
            current_edge = next((x for x in self.mission.Edges if x["Source"]["$ref"] == self.current_waypoint["$id"]), None)
            if(current_edge == None):
                rospy.loginfo("Did not find an outgoing edge! Might be the last one")
                return
            self.current_waypoint = self.find_object_with_id(self.mission.Waypoints, current_edge["Target"])

        # We have the target waypoint. Command the position!
        rospy.loginfo("Commanding point")
        self.command_robot_to(self.current_waypoint)

    # receives the current position of the robot and compares it to the current target.
    # Triggers a state machine step if the target is reached
    def robot_position_updated(self, data):

        if(self.current_anchor_id == None):
            # we are still waiting for the other anchors to be found
            return

        # lookup transform relative to current_anchor
        #if(data.header.frame_id != self.current_anchor_id):
        transform = self.tf_Buffer.lookup_transform(self.current_anchor_id + "_rot", "body", rospy.Time(0), rospy.Duration(0.1))
        data.pose.pose.position.x = transform.transform.translation.x
        data.pose.pose.position.y = transform.transform.translation.y
        data.pose.pose.position.z = transform.transform.translation.z
        data.pose.pose.orientation.x = transform.transform.rotation.x
        data.pose.pose.orientation.y = transform.transform.rotation.y
        data.pose.pose.orientation.z = transform.transform.rotation.z
        data.pose.pose.orientation.w = transform.transform.rotation.w


        pos_t = self.current_goal_pose.position
        ori_t = self.current_goal_pose.orientation
        pos = data.pose.pose.position
        ori = data.pose.pose.orientation
            
        distance_err = self.CalcL2([pos.x, pos.y], [pos_t.x, pos_t.y])
        rotation_err = 0

        if(self.options["orientation_check_enabled"]):
            #Quaternions represent an equal rotation if q1 = q2, or if q1 = -q2
            rotation_err_1 = self.CalcL2([ori.x, ori.y, ori.z, ori.w], [ori_t.x, ori_t.y, ori_t.z, ori_t.w])
            rotation_err_2 = self.CalcL2([ori.x, ori.y, ori.z, ori.w], [ori_t.x, ori_t.y, ori_t.z, ori_t.w], -1)
            rotation_err = rotation_err_1 if rotation_err_1 < rotation_err_2 else rotation_err_2

        #rospy.loginfo("Target: %s", pos_t)
        #rospy.loginfo("Current: %s", pos)
        #rospy.loginfo("Target rot: %s", ori_t)
        #rospy.loginfo("Current rot: %s", ori)
        #rospy.loginfo("Current distance is %s", distance_err)
        #rospy.loginfo("Current rotation error is %s", rotation_err)

        if(distance_err < self.options["tolerance_translation"] and rotation_err < self.options["tolerance_rotation"] ):
            rospy.sleep(2)
            self.perform_step()

    def CalcL2(self, arr1, arr2, factor = 1):
        square = 0
        for i in range(len(arr1)):
            square += (arr1[i]-arr2[i]*factor)**2
        return square**0.5


if __name__ == '__main__':

    executer = MissionExecuter()
    executer.spin()
