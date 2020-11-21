#! /usr/bin/env python

# dependencies:
#   - vehicle properties
#   - odom_gt ros msg

import sys
import rospy
from args_parser.args_parser import ArgsParser
from veh_srv_ros_types.msg import VehCfg
sys.path.append('.')
from odom_gt import OdomGt, OdomGtSim

def update_data(data):
    gaz_frame_id = data.veh_gaz_frame_id.data
    gaz_model_name = data.veh_gaz_model_name.data
    topic_name = data.veh_odomgt_topic_name.data
    gaz_srv_name = data.veh_odomgt_gaz_srv_name.data
    return gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name

def main():
    args_parser = ArgsParser()
    args = args_parser.vehSrvCfg_parse_args()
    if args is None:
        rospy.init_node('audibot_odom_gt', anonymous=True)
        odom_type = 'real'
    else:
        odom_type = args[0]
        if odom_type == 'sim':
            rospy.init_node('audibot_odom_gt_sim', anonymous=True)
        elif odom_type == 'real':
            rospy.init_node('audibot_odom_gt', anonymous=True)
    data = rospy.wait_for_message('audibot/veh/cfg', VehCfg, timeout=None)
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