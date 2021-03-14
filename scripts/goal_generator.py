#!/usr/bin/python
import rospy

import math
import tf2_ros
from geometry_msgs.msg import PoseStamped
if __name__ == '__main__':
    rospy.init_node('goal_generator')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    goal_pose_pub = rospy.Publisher('/laikago_tracker/goal',PoseStamped,queue_size=10)
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'target', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        target_pose = PoseStamped()
        # target_pose.header.frame_id = 'world'
        # target_pose.header.frame_id = trans.header.frame_id
        target_pose.pose.position.x = trans.transform.translation.x
        target_pose.pose.position.y = trans.transform.translation.y
        target_pose.pose.position.z = trans.transform.translation.z
        target_pose.pose.orientation.x = trans.transform.rotation.x
        target_pose.pose.orientation.y = trans.transform.rotation.y
        target_pose.pose.orientation.z = trans.transform.rotation.z
        target_pose.pose.orientation.w = trans.transform.rotation.w

        # rospy.loginfo("%f,%f,%f",trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z)

        goal_pose_pub.publish(target_pose)

        rate.sleep()