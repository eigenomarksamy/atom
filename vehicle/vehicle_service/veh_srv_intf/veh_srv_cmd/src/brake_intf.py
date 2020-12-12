#! /usr/bin/env python

import sys
import rospy
from veh_srv_ros_types.msg import VehBrake
sys.path.append('.')
from cmd_intf import Cmd

class Brake(Cmd):
    def __init__(self, sub_topic_name, pub_topic_name=None):
        Cmd.__init__(self, sub_topic_name, pub_topic_name)
        self.init()

    def execute(self):
        self._data = rospy.wait_for_message(topic=self._sub_topic_name, topic_type=VehBrake)
        self.fill_pub_msg(self._data)
        self.publish()

def main():
    sub_topic_name = None
    pub_topic_name = None
    obj = Brake(sub_topic_name=sub_topic_name, pub_topic_name=pub_topic_name)
    obj.init()
    rate_obj, _ = obj.get_rate_props()
    while not rospy.is_shutdown:
        obj.execute()
        rate_obj.sleep()

if __name__ == "__main__":
    main()