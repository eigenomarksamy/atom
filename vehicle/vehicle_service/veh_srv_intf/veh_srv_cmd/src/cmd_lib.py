#! /usr/bin/env python

import rospy
from args_parser.args_parser import ArgsParser
from veh_cfg.veh_cfg import VehSrvComCfg
from std_msgs.msg import Float64, UInt8

class Cmd:
    def __init__(self, sub_topic_name, pub_topic_name):
        self._sub_topic_name = sub_topic_name
        self._pub_topic_name = pub_topic_name
        self._rate_val = 100

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

class CmdNodeProps:

    def parse_cmd_args(self):
        self._args_parser = ArgsParser()
        self._args_dict = self._args_parser.get_veh_launch_params()
        self._ns = self._args_dict['veh_params']['ns']
        self._str_len = 1
        self._veh_name = self._args_dict['veh_params']['name']
        self._veh_id = self._args_dict['veh_params']['id']

    def get_throt_node_props(self):
        com_cfg_obj = VehSrvComCfg(self._ns, self._str_len, self._veh_name, self._veh_id)
        self._throt_node_name   = com_cfg_obj._throt_node_name
        self._throt_topic_in    = com_cfg_obj._throt_topic_in_name
        self._throt_topic_out   = com_cfg_obj._throt_topic_out_name
        self._throt_pub_rate    = com_cfg_obj._throt_pub_rate
        return self._throt_node_name, self._throt_topic_in, self._throt_topic_out, self._throt_pub_rate

    def get_brake_node_props(self):
        com_cfg_obj = VehSrvComCfg(self._ns, self._str_len, self._veh_name, self._veh_id)
        self._brake_node_name   = com_cfg_obj._brake_node_name
        self._brake_topic_in    = com_cfg_obj._brake_topic_in_name
        self._brake_topic_out   = com_cfg_obj._brake_topic_out_name
        self._brake_pub_rate    = com_cfg_obj._brake_pub_rate
        return self._brake_node_name, self._brake_topic_in, self._brake_topic_out, self._brake_pub_rate

    def get_steer_node_props(self):
        com_cfg_obj = VehSrvComCfg(self._ns, self._str_len, self._veh_name, self._veh_id)
        self._steer_node_name   = com_cfg_obj._steer_node_name
        self._steer_topic_in    = com_cfg_obj._steer_topic_in_name
        self._steer_topic_out   = com_cfg_obj._steer_topic_out_name
        self._steer_pub_rate    = com_cfg_obj._steer_pub_rate
        return self._steer_node_name, self._steer_topic_in, self._steer_topic_out, self._steer_pub_rate

    def get_gear_node_props(self):
        com_cfg_obj = VehSrvComCfg(self._ns, self._str_len, self._veh_name, self._veh_id)
        self._gear_node_name   = com_cfg_obj._gear_node_name
        self._gear_topic_in    = com_cfg_obj._gear_topic_in_name
        self._gear_topic_out   = com_cfg_obj._gear_topic_out_name
        self._gear_pub_rate    = com_cfg_obj._gear_pub_rate
        return self._gear_node_name, self._gear_topic_in, self._gear_topic_out, self._gear_pub_rate