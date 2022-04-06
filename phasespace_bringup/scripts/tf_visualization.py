#! /usr/bin/env python3

import rospy
import tf

from phasespace_msgs.msg import *

base_name = "phasespace_base"

def handle_rigid_pose(msg):
	br = tf.TransformBroadcaster()
	for i in range(len(msg.rigids)):
		n_msg = msg.rigids[i]
		br.sendTransform((n_msg.x, n_msg.y, n_msg.z), [n_msg.qx, n_msg.qy, n_msg.qz, n_msg.qw], rospy.Time.now(), "robot_id_{}".format(i), base_name)

def handle_camera_pose(msg):
	br = tf.TransformBroadcaster()
	# transform = tf.Transform()
	for i in range(len(msg.cameras)):
		n_msg = msg.cameras[i]
		br.sendTransform((n_msg.x, n_msg.y, n_msg.z), [n_msg.qx, n_msg.qy, n_msg.qz, n_msg.qw], rospy.Time.now(), "camera_{}".format(i), base_name)

if __name__ == '__main__':
	rospy.init_node('tf_visualization')
	rospy.Subscriber('/phasespace/rigids', Rigids, handle_rigid_pose)
	rospy.Subscriber('/phasespace/cameras', Cameras, handle_camera_pose)
	rospy.spin()


# x -> z
# y -> x
# z -> y