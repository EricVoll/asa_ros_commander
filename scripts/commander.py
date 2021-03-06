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

import sys


class AsaCommander:


    def __init__(self):

        rospy.init_node('asa_commander', anonymous=True)

        # check arguments
        myargv = rospy.myargv(argv = sys.argv)
        if(len(myargv) < 4):
            rospy.logerr("Missing argument for the commander.py script! Usage 'robot_name:string' 'mocking_anchors:bool' 'command_delay:float'")

        self.options = {
            "robot_name": myargv[1],
            "mocking_anchors": myargv[2] == 'True',
            "command_delay": float(myargv[3])
        }
        
        #tf setup
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.root_frame = "odom"
        self.current_anchor_id = self.root_frame
        self.rate = rospy.Rate(10.0)
        self.anchors = []
        self.last_closest_anchor = None
        self.mocking = self.options["mocking_anchors"]
        self.robot_used = self.options["robot_name"] # {"jackal", "spot"} mainly has influence on which mechanism is used to re-publish the received goal
        
        if(self.robot_used == "jackal"):
            self.robot_frame = "base_link"
        elif(self.robot_used =="spot"):
            self.robot_frame = "body"
        
        # All publishers
        self.odom_publisher = rospy.Publisher('/odometry/filtered/asa_relative', Odometry, queue_size=10)
        self.move_base_simple_publisher = None
        self.move_base_cancel_publisher = None
        
        #add all subscribers needed
        rospy.Subscriber('/anchored_goal', AsaRelPoseStamped, self.anchored_goal_callback)
        if(self.robot_used =="jackal"):
            rospy.Subscriber('/odometry/filtered', Odometry, self.republish_robot_pos)
        elif(self.robot_used == "spot"):
            rospy.Subscriber('spot/odometry', Odometry, self.republish_robot_pos)

        # Subscribers listening to the asa ros wrapper
        rospy.Subscriber('/asa_ros/found_anchor', FoundAnchor, self.asa_found_anchor_callback)
        rospy.Subscriber('/asa_ros/created_anchor', CreatedAnchor, self.asa_created_anchor_callback)

        rospy.loginfo("Initialized asa_ros_commander")       

    def spin(self):
        rospy.spin()




#########
# Utils #
#########

    # applies a constant rotation to the frame relative to odom
    def republish_frame_rotated(self, frame_name):
        if(self.mocking):
            return frame_name
        rotatedFrameName = frame_name + "_rot"
        rotatedtrans = self.tf_Buffer.lookup_transform("odom", frame_name, rospy.Time(0), rospy.Duration(0.1))
        rotation = rotatedtrans.transform.rotation

        quat1 = [-0.5, 0.5, 0.5, 0.5]
        quat2 = [rotation.x,rotation.y,rotation.z,rotation.w]

        nr = quaternion_multiply(quat2, quat1)

        rotatedtrans.transform.rotation.x = nr[0]
        rotatedtrans.transform.rotation.y = nr[1]
        rotatedtrans.transform.rotation.z = nr[2]
        rotatedtrans.transform.rotation.w = nr[3]
        rotatedtrans.child_frame_id = rotatedFrameName
        self.tf_static_broadcaster.sendTransform(rotatedtrans)

        return rotatedFrameName


    # transforms a post to a transform object
    def pose_to_tf(self, pose, frame_name, parent_frame, time=None):
        """
        Generate a TF from a given pose, frame, and parent.
        """
        assert pose is not None, 'Cannot have None for pose.'
        _tf = TransformStamped()
        
        _tf.child_frame_id = frame_name
        if time is None:
            time = rospy.Time.now()
        _tf.header.stamp = time
        _tf.header.frame_id = parent_frame

        _tf.transform.translation = pose.position
        _tf.transform.rotation = pose.orientation

        return _tf

#####################
# Anchor management #
#####################

    # Adds the found anchors id to the available id list
    def asa_found_anchor_callback(self, data):
        rospy.loginfo(data)
        self.add_asa_frame(data.anchor_id)

    # Adds the created anchors id to the available id list if it succeeded
    def asa_created_anchor_callback(self, data):
        
        mocking = True
        if(mocking):
            rospy.loginfo("Did add anchor from created anchor since I am in debug mode")
        else:
            rospy.loginfo("Did not add created anchor, since no tf frame was created from that")
            return

        # if we wanted to add it here, that would be the code:
        if(data.success):
            self.add_asa_frame(data.anchor_id)
        else:
            rospy.loginfo("Asa Anchor Creation failed")



    # Adds an anchor to the anchor list and overwrites the default if none is set
    def add_asa_frame(self, anchor_id):

        if(anchor_id == "mocked_0"):
            #reset the list
            self.anchors = []

        if(len(self.anchors) == 0):
            rospy.loginfo("Added first anchor, id = " + anchor_id)

        # republish all incoming frames with their rotated version, since they'll be used from now on.
        self.republish_frame_rotated(anchor_id)

        #overwrite the default the first time we receive an anchor
        if self.current_anchor_id == self.root_frame:
            self.current_anchor_id = anchor_id
        if anchor_id not in self.anchors:
            self.anchors.append(anchor_id)
            rospy.loginfo("added id " + anchor_id + " to anchor list")


    # Queries all registered anchors and returns the id of the nearest one to the robot frame
    def find_nearest_anchor(self):

        if(len(self.anchors) == 1):
            return self.anchors[0]

        smallestDistance = 1000000
        smallestDistanceAnchorId = None
        for anchor in self.anchors:
            distance = self.getDistanceToAnchor(anchor, self.robot_frame)
            if(distance < smallestDistance):
                smallestDistance = distance
                smallestDistanceAnchorId = anchor

        # Check if the closest anchor changed and notify terminal about it.
        if(self.last_closest_anchor != smallestDistanceAnchorId):
            rospy.loginfo("changed anchor to " + smallestDistanceAnchorId)
            self.last_closest_anchor = smallestDistanceAnchorId

        return smallestDistanceAnchorId

    # returns the distance between the two frames
    def getDistanceToAnchor(self, anchor_id, target_frame):
        trans = self.tf_Buffer.lookup_transform(target_frame, anchor_id, rospy.Time(0))
        tr = trans.transform.translation
        return (tr.x**2 + tr.y**2 + tr.z**2)**0.5

################
# Republishers #
################

    def anchored_goal_callback(self, data):
        if(self.mocking):
            current_anchor_id = data.anchor_id
        else:
            current_anchor_id = data.anchor_id + "_rot"
        rospy.loginfo(rospy.get_caller_id() + ' Commanding robot relative to the anchor with id %s', current_anchor_id)

        # create current target frame
        current_goal_frame_id = "current_goal"
        try:
            transform = self.pose_to_tf(data.pose, current_goal_frame_id, current_anchor_id)
        except:
            rospy.loginfo("Could not find transform to anchor_id: " + current_anchor_id)
            return
            
        self.tf_broadcaster.sendTransform(transform)
        
        rospy.sleep(0.3)

        target_frame = self.root_frame
        if(self.robot_used == "spot"):
            target_frame = self.robot_frame # spot interprets commands realtive to the body.

        # lookup transform to odom
        try:
            trans = self.tf_Buffer.lookup_transform(target_frame, current_goal_frame_id, rospy.Time(0), rospy.Duration(1))
            rospy.loginfo("found tf: %s", trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Failed to lookup anchor for anchored goal once.")
            return

            
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.root_frame
        target_pose.pose.position.x = trans.transform.translation.x
        target_pose.pose.position.y = trans.transform.translation.y
        target_pose.pose.position.z = trans.transform.translation.z
        target_pose.pose.orientation.x = trans.transform.rotation.x
        target_pose.pose.orientation.y = trans.transform.rotation.y
        target_pose.pose.orientation.z = trans.transform.rotation.z
        target_pose.pose.orientation.w = trans.transform.rotation.w

        # republish the StampedPose relative to the base frame
        if(self.robot_used == "spot"):
            # use spot wrapper trajectory service
 
            # build trajectory command
            rospy.loginfo("Marked goal. Sleep.")
            rospy.sleep(self.options["command_delay"])
            rospy.loginfo("Starting command.")
            trajectory = Trajectory()
            trajectory.target_pose = target_pose
            trajectory.target_pose.header.frame_id = "body"
            trajectory.duration = 15
            
            rospy.wait_for_service('spot/trajectory')
            trajectory_service = rospy.ServiceProxy('spot/trajectory', Trajectory)
            response = trajectory_service(trajectory.target_pose, trajectory.duration)
            rospy.loginfo(response.success)
            rospy.loginfo(response.message)

        elif (self.robot_used == "jackal"):
            ## use movebase simple goal publisher
            if(self.move_base_simple_publisher == None):
                self.move_base_simple_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
                self.move_base_cancel_publisher = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)

            # create StampedPose from transform
            self.move_base_cancel_publisher.publish(GoalID())
            rospy.sleep(self.options["command_delay"])

            self.move_base_simple_publisher.publish(target_pose)


    # Republishes the received omodemetry relative to the odom frame but relative to the nearest anchor available
    def republish_robot_pos(self, data):
        current_parent_frame_id = data.header.frame_id
        child_frame_id = data.child_frame_id

        if(current_parent_frame_id == self.current_anchor_id):
            #rospy.loginfo("Skipped republishing since current anchor id "+ self.current_anchor_id + " is same as sent parent")
            return

        self.current_anchor_id = self.find_nearest_anchor()

        #create rotated anchor
        if (self.mocking):
            rotatedFrameName = self.current_anchor_id
        else:
            rotatedFrameName = self.current_anchor_id + "_rot"

        #Lookup the transform relative to the 
        trans = self.tf_Buffer.lookup_transform(rotatedFrameName, child_frame_id, rospy.Time(0), rospy.Duration(1.0))
        new_odom = Odometry()
        new_odom.header = data.header
        new_odom.header.frame_id = self.current_anchor_id

        #Todo: couldn't find the data type of position. Will make nicer in future.
        new_odom.pose.pose.position.x = trans.transform.translation.x
        new_odom.pose.pose.position.y = trans.transform.translation.y
        new_odom.pose.pose.position.z = trans.transform.translation.z
        new_odom.pose.pose.orientation.x = trans.transform.rotation.x
        new_odom.pose.pose.orientation.y = trans.transform.rotation.y
        new_odom.pose.pose.orientation.z = trans.transform.rotation.z
        new_odom.pose.pose.orientation.w = trans.transform.rotation.w
        new_odom.pose.covariance = data.pose.covariance #We are no using covariance at all. Maybe do not set it?

        self.odom_publisher.publish(new_odom)


if __name__ == '__main__':



    commander = AsaCommander()
    commander.spin()
