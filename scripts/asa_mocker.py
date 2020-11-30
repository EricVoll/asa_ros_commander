#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
import tf2_ros as tf
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from asa_ros_commander.msg import CreatedAnchorMock, FoundAnchorMock, AsaRelPoseStamped
import time

class AsaAnchorMocker:


    def __init__(self):
        rospy.init_node('asa_anchor_mocker', anonymous=True)
        self.tf_broadcaster = tf.StaticTransformBroadcaster()
        self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        self.rate = rospy.Rate(10.0)
        self.anchors = []
        self.found_anchor_pub = rospy.Publisher('/asa_ros/found_anchor', FoundAnchorMock, queue_size=10)
        self.created_anchor_pub = rospy.Publisher('/asa_ros/created_anchor', CreatedAnchorMock, queue_size=10)
        self.world_frame_id = "odom"

        #add all subscribers needed
        rospy.Subscriber('/mock_anchor_found', AsaRelPoseStamped, self.mock_anchor_callback_found)
        rospy.Subscriber('/mock_anchor_created', AsaRelPoseStamped, self.mock_anchor_callback_created)

        rospy.loginfo("Initialized asa_mocker")       

    def spin(self):
        rospy.spin()

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

    # Creates a tf frame at the defined location, mocks the asa ros wrappers published msg
    def mock_anchor_callback_created(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' Mocking anchor created '+ data.anchor_id  +' at location %s', data.pose)
        transform = self.pose_to_tf(data.pose, data.anchor_id, self.world_frame_id, rospy.Time.now())
        self.tf_broadcaster.sendTransform(transform)

        newAnchor = CreatedAnchorMock()
        newAnchor.success = True
        newAnchor.anchor_id = data.anchor_id
        newAnchor.failure_reason = ""

        self.created_anchor_pub.publish(newAnchor)

        
    # Creates a tf frame at the defined location, mocks the asa ros wrappers published msg
    def mock_anchor_callback_found(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' Mocking anchor ound '+ data.anchor_id  +' at location %s', data.pose)
        transform = self.pose_to_tf(data.pose, data.anchor_id, self.world_frame_id, rospy.Time.now())
        self.tf_broadcaster.sendTransform(transform)

        newAnchor = FoundAnchorMock()
        newAnchor.anchor_id = data.anchor_id

        self.found_anchor_pub.publish(newAnchor)



if __name__ == '__main__':
    rospy.loginfo("Starting asa_mocker")
    mocker = AsaAnchorMocker()
    mocker.spin()
