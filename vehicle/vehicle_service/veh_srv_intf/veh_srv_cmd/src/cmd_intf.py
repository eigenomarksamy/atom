#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64, UInt8

class Cmd:
    def __init__(self, sub_topic_name, pub_topic_name):
        self._sub_topic_name = sub_topic_name
        self._pub_topic_name = pub_topic_name
        self._rate_val = 100
        self._data = None

    def init(self):
        self._pub = rospy.Publisher(self._pub_topic_name, Float64, queue_size=100)
        self._pub_msg = Float64()
        self._rate_obj = rospy.Rate(self._rate_val)

    def set_rate_val(self, rate_val=100):
        self._rate_val = rate_val

    def get_rate_props(self):
        return self._rate_obj, self._rate_val

    def fill_pub_msg(self, data):
        self._pub_msg = data

    def publish(self):
        self._pub.publish(self._pub_msg)

    def get_data(self):
        return self._data

class GearCmd(Cmd):
    def __init__(self, sub_topic_name, pub_topic_name):
        Cmd.__init__(self, sub_topic_name, pub_topic_name)

    def init(self):
        self._pub = rospy.Publisher(self._pub_topic_name, UInt8, queue_size=100)
        self._pub_msg = UInt8()
        self._rate_obj = rospy.Rate(self._rate_val)