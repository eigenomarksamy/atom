#! /usr/bin/env python

# dependencies:
#   - vehicle properties
#   - odom_gt ros msg

import sys
import rospy
sys.path.append('.')
sys.path.append('../../../veh_srv_cfg/veh_cfg/scripts')
from odom_gt import OdomGt
from veh_srv_cfg import VehSrvCfgOdomGt

def main():
    veh_cfg = VehSrvCfgOdomGt(vehNs='audibot')
    odom_gt = OdomGt(veh_cfg)
    odom_gt.init()
    odom_pub_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        odom_gt.execute()
        odom_pub_rate.sleep()

if __name__ == "__main__":
    main()