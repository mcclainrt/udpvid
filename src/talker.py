#!/usr/bin/env python
"""Publish data to ROS topic
"""
import rospy
from std_msgs.msg import String

topic = 'chatter'
#pub = rospy.Publisher(topic, String)
rospy.init_node('talker', anonymous=True)
rospy.loginfo("I will publish to the topic %s", topic)
while not rospy.is_shutdown():
	str = "hello world %s"%rospy.get_time()
	rospy.loginfo(str)
	#pub.publish(str)
	rospy.sleep(0.1)
