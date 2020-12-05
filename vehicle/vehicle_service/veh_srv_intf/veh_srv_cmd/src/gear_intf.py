#! /usr/bin/env python

import sys
import rospy
from veh_srv_ros_types.msg import VehGear
from std_msgs.msg import UInt8

class Gear:
    def __init__(self, sub_topic_name, pub_topic_name):
        self._sub_topic_name = sub_topic_name
        self._pub_topic_name = pub_topic_name
        self._data = None
        self._rate = 100

    def init(self):
        self._pub = rospy.Publisher(self._pub_topic_name, UInt8, queue_size=100)
        self._pub_msg = UInt8()
        self._rate_ros = rospy.Rate(self._rate)

    def fill_pub_msg(self, data):
        self._pub_msg = data

    def publish(self):
        self._pub.publish(self._pub_msg)

    def set_rate(self, rate=100):
        self._rate = rate

    def get_rate(self):
        return self._rate_ros, self._rate

    def get_data(self):
        return self._data

    def execute(self):
        self._data = rospy.wait_for_message(topic=self._sub_topic_name, topic_type=VehGear)
        self.fill_pub_msg(self._data)
        self.publish()

def main():
    sub_topic_name = None
    pub_topic_name = None
    obj = Gear(sub_topic_name=sub_topic_name, pub_topic_name=pub_topic_name)
    obj.set_rate()
    obj.init()
    rate_obj, _ = obj.get_rate()
    while not rospy.is_shutdown:
        obj.execute()
        rate_obj.sleep()

if __name__ == "__main__":
    main()