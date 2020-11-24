#! /usr/bin/env python

import rospy
from veh_cfg.veh_cfg import VehSrvComCfg
from veh_srv_ros_types.msg import VehComCfg

def init_ros(veh_cfg_obj):
    rospy.init_node(veh_cfg_obj._cfg_node_name)
    pub = rospy.Publisher(veh_cfg_obj._cfg_topic_name, VehComCfg, queue_size=10)
    return pub

def add_initial_cfg(veh_cfg_obj):
    pub_msg = VehComCfg()
    pub_msg.timestamp = rospy.Time()
    pub_msg.seq_counter = 1
    pub_msg.veh_ns.data = veh_cfg_obj._pub_vehNs
    pub_msg.veh_str_len.data = veh_cfg_obj._pub_vehStrLen
    pub_msg.veh_name.data = veh_cfg_obj._pub_vehName
    pub_msg.veh_id.data = veh_cfg_obj._pub_vehId
    pub_msg.veh_gaz_model_name.data = veh_cfg_obj._pub_gazModelName
    pub_msg.veh_gaz_frame_id.data = veh_cfg_obj._pub_gazFrameId
    pub_msg.veh_odomgt_node_name.data = veh_cfg_obj._pub_odomgtNodeName
    pub_msg.veh_odomgt_topic_name.data = veh_cfg_obj._pub_odomgtTopicName
    pub_msg.veh_odomgt_gaz_srv_name.data = veh_cfg_obj._pub_odomgtGazSrvName
    pub_msg.veh_cfg_node_name.data = veh_cfg_obj._pub_cfgNodeName
    pub_msg.veh_cfg_topic_name.data = veh_cfg_obj._pub_cfgTopicName
    return pub_msg

def main():
    veh_cfg_obj = VehSrvComCfg('audibot')
    pub = init_ros(veh_cfg_obj)
    print("Veh Com Cfg Publisher Node")
    pub_msg = add_initial_cfg(veh_cfg_obj)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(pub_msg)
        rate.sleep()

if __name__ == '__main__':
    main()