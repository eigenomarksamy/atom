#! /usr/bin/env python

# dependencies:
#   - vehicle properties
#   - odom_gt ros msg

import sys
import rospy
from args_parser.args_parser import ArgsParser
from veh_srv_ros_types.msg import VehComCfg
from veh_cfg.veh_cfg import VehSrvComCfg
sys.path.append('.')
from odom_gt import OdomGt, OdomGtSim

def update_data(data):
    gaz_frame_id = data.veh_gaz_frame_id.data
    gaz_model_name = data.veh_gaz_model_name.data
    topic_name = data.veh_odomgt_topic_name.data
    gaz_srv_name = data.veh_odomgt_gaz_srv_name.data
    return gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name

def main():
    args_parser_odomgt = ArgsParser()
    args_dict = args_parser_odomgt.get_veh_launch_params()
    veh_ns = args_dict['veh_params']['ns']
    veh_srv_com_cfg = VehSrvComCfg(veh_ns)
    args = args_parser_odomgt.get_args()
    if args.sim:
        rospy.init_node(veh_srv_com_cfg._pub_odomgtNodeName + '_sim')
        print("ODOM GT Sim Publisher Node")
        odom_type = 'sim'
    else:
        rospy.init_node(veh_srv_com_cfg._pub_odomgtNodeName)
        print("ODOM GT Publisher Node")
        odom_type = 'real'
    data = rospy.wait_for_message(veh_srv_com_cfg._pub_cfgTopicName, VehComCfg, timeout=None)
    gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name = update_data(data)
    if odom_type == 'sim':
        odom_gt = OdomGtSim(gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name)
    else:
        odom_gt = OdomGt(gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name)
    odom_gt.init()
    odom_pub_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        odom_gt.execute()
        odom_pub_rate.sleep()

if __name__ == "__main__":
    main()