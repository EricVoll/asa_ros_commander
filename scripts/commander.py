#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
import tf2_ros as tf
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from asa_ros_commander.msg import AsaRelPoseStamped
import time

class AsaCommander:

    #current_anchor_id = "odom"
    #tf_broadcaster = None
    #tf_listener = None
    #odom_publisher = None

    def __init__(self):
        rospy.init_node('asa_commander', anonymous=True)
        self.tf_broadcaster = tf.StaticTransformBroadcaster()
        self.tf_Buffer = tf.Buffer(rospy.Duration(120))
        self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.odom_publisher = rospy.Publisher('/odometry/filtered/asa_relative', Odometry, queue_size=10)
        self.current_anchor_id = "odom"
        self.rate = rospy.Rate(10.0)
        self.anchors = []

        #add all subscribers needed
        rospy.Subscriber('/mock_anchor', AsaRelPoseStamped, self.mock_anchor_callback)
        rospy.Subscriber('/anchored_goal', AsaRelPoseStamped, self.anchored_goal_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.republish_robot_pos)

        rospy.loginfo("Initialized asa_ros_commander")       

    def spin(self):
        rospy.spin()

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

    def anchored_goal_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' Commanding robot relative to the anchor with id %s', data.anchor_id)

    # Creates an anchor at the defined location relative to odom
    def mock_anchor_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' received msg')
        rospy.loginfo(rospy.get_caller_id() + ' Mocking anchor '+ data.anchor_id  +' at location %s', data.pose)
        transform = self.pose_to_tf(data.pose, data.anchor_id, "odom", rospy.Time.now())
        self.tf_broadcaster.sendTransform(transform)
        self.set_current_asa_frame(data.anchor_id)


    def set_current_asa_frame(self, anchor_id):
        #overwrite the default the first time we receive an anchor
        if self.current_anchor_id == "odom":
            self.current_anchor_id = anchor_id
        if anchor_id not in self.anchors:
            self.anchors.append(anchor_id)
            rospy.loginfo("added id " + anchor_id + " to anchor list")

    def republish_robot_pos(self, data):
        current_parent_frame_id = data.header.frame_id
        child_frame_id = data.child_frame_id

        if(current_parent_frame_id == self.current_anchor_id):
            rospy.loginfo("Skipped republishing since current anchor id "+ self.current_anchor_id + " is same as sent parent")
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

    def find_nearest_anchor(self):

        if(len(self.anchors) == 1):
            return self.anchors[0]

        smallestDistance = 1000000
        smallestDistanceAnchor = None
        for anchor in self.anchors:
            distance = self.getDistanceToAnchor(anchor, "base_link")
            if(distance < smallestDistance):
                smallestDistance = distance
                smallestDistanceAnchor = anchor

        rospy.loginfo("Smallest distance is: %s" , smallestDistance)
        return smallestDistanceAnchor

    def getDistanceToAnchor(self, anchor_id, target_frame):
        trans = self.tf_Buffer.lookup_transform(target_frame, anchor_id, rospy.Time(0))
        tr = trans.transform.translation
        return (tr.x**2 + tr.y**2 + tr.z**2)**0.5

if __name__ == '__main__':
    commander = AsaCommander()
    commander.spin()
