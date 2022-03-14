#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Pose
import datetime
import csv
from nav_msgs.msg import Path

class Vis_Path(object):
	def __init__(self):
		self.pub_path = rospy.Publisher('our', Path, queue_size=10)
		self.path = Path()
		self.path.header.frame_id = 'map'
		odom_sub = rospy.Subscriber("truth_map_posestamped", PoseStamped, self.cb_pose, queue_size=10)

	def cb_stick(self,msg):
		self.push = msg.data

	def cb_pose(self,msg):
		if msg.pose.position.x==0 and msg.pose.position.y==0 and msg.pose.position.z==0:return
		self.path.poses.append(msg)
		self.pub_path.publish(self.path)
		rospy.loginfo((msg.pose.position.x,msg.pose.position.y,msg.pose.position.z))


if __name__ == "__main__":
	rospy.init_node("Vis_Path")
	wheeltf = Vis_Path()
	rospy.spin()
