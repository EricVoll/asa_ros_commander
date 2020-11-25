#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
import tf2_ros as tf
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from asa_ros_commander.msg import AsaRelPoseStamped
from asa_ros_commander.msg import CreatedAnchorMock as CreatedAnchor
from asa_ros_commander.msg import FoundAnchorMock as FoundAnchor
import time

class AsaCommander:


    def __init__(self):
        rospy.init_node('asa_commander', anonymous=True)
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.odom_publisher = rospy.Publisher('/odometry/filtered/asa_relative', Odometry, queue_size=10)
        self.current_anchor_id = "odom"
        self.rate = rospy.Rate(10.0)
        self.anchors = []
        self.robot_frame = "base_link"

        #add all subscribers needed
        rospy.Subscriber('/anchored_goal', AsaRelPoseStamped, self.anchored_goal_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.republish_robot_pos)

        # Subscribers listening to the asa ros wrapper
        rospy.Subscriber('/found_anchor', FoundAnchor, self.asa_found_anchor_callback)
        rospy.Subscriber('/created_anchor', CreatedAnchor, self.asa_created_anchor_callback)

        rospy.loginfo("Initialized asa_ros_commander")       

    def spin(self):
        rospy.spin()

#########
# Utils #
#########


#####################
# Anchor management #
#####################

    # Adds the found anchors id to the available id list
    def asa_found_anchor_callback(self, data):
        self.add_asa_frame(data.anchor_id)

    # Adds the created anchors id to the available id list if it succeeded
    def asa_created_anchor_callback(self, data):
        if(data.success):
            self.add_asa_frame(data.anchor_id)
        else:
            rospy.loginfo("Asa Anchor Creation failed")



    # Adds an anchor to the anchor list and overwrites the default if none is set
    def add_asa_frame(self, anchor_id):
        #overwrite the default the first time we receive an anchor
        if self.current_anchor_id == "odom":
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
        rospy.loginfo(rospy.get_caller_id() + ' Commanding robot relative to the anchor with id %s', data.anchor_id)


    # Republishes the received omodemetry relative to the odom frame but relative to the nearest anchor available
    def republish_robot_pos(self, data):
        current_parent_frame_id = data.header.frame_id
        child_frame_id = data.child_frame_id

        if(current_parent_frame_id == self.current_anchor_id):
            #rospy.loginfo("Skipped republishing since current anchor id "+ self.current_anchor_id + " is same as sent parent")
            return

        self.current_anchor_id = self.find_nearest_anchor()

        #Lookup the transform relative to the 
        try:
            #lookup takes about .1ms
            trans = self.tf_Buffer.lookup_transform(child_frame_id, self.current_anchor_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.rate.sleep()
            rospy.loginfo("failed once")
            return
        new_odom = Odometry()
        new_odom.header = data.header
        new_odom.header.frame_id = self.current_anchor_id

        #Todo: couldn't find the data type of position. Will make nicer in future.
        new_odom.pose.pose.position.x = trans.transform.translation.x
        new_odom.pose.pose.position.y = trans.transform.translation.y
        new_odom.pose.pose.position.z = trans.transform.translation.z
        new_odom.pose.pose.orientation = Quaternion()
        new_odom.pose.covariance = data.pose.covariance #We are no using covariance at all. Maybe do not set it?

        rospy.loginfo("Republished odom relative to " + self.current_anchor_id + "!")

        self.odom_publisher.publish(new_odom)


if __name__ == '__main__':
    commander = AsaCommander()
    commander.spin()
