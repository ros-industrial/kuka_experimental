#!/usr/bin/python

import tf2_ros
import tf2_geometry_msgs
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node("help")
tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

while (not rospy.is_shutdown()):
    try:
        transform = tf_buffer.lookup_transform("link_3",
                                               # source frame:
                                               "base_link",
                                               # get the tf at the time the pose was valid
                                               rospy.Time.now(),
                                               # wait for at most 1 second for transform, otherwise throw
                                               rospy.Duration(1.0))

        pose = PoseStamped()
        pose.pose.position.x=1.1985
        pose.pose.position.y=0
        pose.pose.position.z=1.91
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
        print(f"transform: {transform.transform.translation}")
        print(f"pose_transformed: {pose_transformed.pose.position}")
    except Exception as e:
        print(e)
    rospy.sleep(1)
