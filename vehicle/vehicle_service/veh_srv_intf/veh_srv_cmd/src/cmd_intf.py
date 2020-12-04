#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

class Cmd:
    def __init__(self, sub_topic_name, pub_topic_name):
        self._sub_topic_name = sub_topic_name
        self._pub_topic_name = pub_topic_name

    def init(self):
        self._pub = rospy.Publisher(self._pub_topic_name, Float64, queue_size=100)
        self._pub_msg = Float64()

    def fill_pub_msg(self, data):
        self._pub_msg = data

    def publish(self):
        self._pub.publish(self._pub_msg)