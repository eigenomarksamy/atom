#! /usr/bin/env python

import sys
import rospy
from veh_srv_ros_types.msg import VehSteering
sys.path.append('.')
from cmd_lib import Cmd, CmdNodeProps

class Steering(Cmd):
    def __init__(self, sub_topic_name, pub_topic_name=None):
        Cmd.__init__(self, sub_topic_name, pub_topic_name)

    def execute(self):
        self._data = rospy.wait_for_message(topic=self._sub_topic_name, topic_type=VehSteering)
        self.fill_pub_msg(self._data.steering_cmd)
        self.publish()

def main():
    print("Veh Steering Node")
    props_obj = CmdNodeProps()
    props_obj.parse_cmd_args()
    node_name, topic_in, topic_out, pub_rate = props_obj.get_steer_node_props()
    rospy.init_node(name=node_name)
    obj = Steering(topic_in, topic_out)
    obj.set_rate_val(pub_rate)
    obj.init()
    rate_obj, _ = obj.get_rate_props()
    while not rospy.is_shutdown():
        obj.execute()

if __name__ == "__main__":
    main()