#! /usr/bin/env python

# dependencies:
#   - vehicle properties
#   - odom_gt ros msg

import rospy
from veh_srv_ros_types.msg import VehOdomGt
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

class OdomGt:
    def __init__(self, gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name):
        self._gaz_frame_id = gaz_frame_id
        self._gaz_model_name = gaz_model_name
        self._topic_name = topic_name
        self._gaz_srv_name = gaz_srv_name

    def init(self):
        self._odom_pub = rospy.Publisher(self._topic_name, VehOdomGt, queue_size=100)
        rospy.wait_for_service(self._gaz_srv_name)
        self._get_model_srv = rospy.ServiceProxy(self._gaz_srv_name, GetModelState)
        self._veh_odom_gt = VehOdomGt()
        self._veh_odom_gt.odometry.header.frame_id = self._gaz_frame_id
        self._model = GetModelStateRequest()
        self._model.model_name = self._gaz_model_name

    def execute(self):
        self._result = self._get_model_srv(self._model)
        self._veh_odom_gt.odometry.pose.pose = self._result.pose
        self._veh_odom_gt.odometry.twist.twist = self._result.twist
        self._veh_odom_gt.odometry.header.stamp = rospy.Time.now()
        self._veh_odom_gt.timestamp = self._veh_odom_gt.odometry.header.stamp
        self._odom_pub.publish(self._veh_odom_gt)

class OdomGtSim(OdomGt):
    def __init__(self, gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name):
        OdomGt.__init__(self, gaz_frame_id, gaz_model_name, topic_name, gaz_srv_name)
        self._topic_name += '/sim'

    def init(self):
        self._odom_pub = rospy.Publisher(self._topic_name, Odometry, queue_size=100)
        rospy.wait_for_service(self._gaz_srv_name)
        self._get_model_srv = rospy.ServiceProxy(self._gaz_srv_name, GetModelState)
        self._veh_odom_gt = Odometry()
        self._veh_odom_gt.header.frame_id = self._gaz_frame_id
        self._model = GetModelStateRequest()
        self._model.model_name = self._gaz_model_name

    def execute(self):
        self._result = self._get_model_srv(self._model)
        self._veh_odom_gt.pose.pose = self._result.pose
        self._veh_odom_gt.twist.twist = self._result.twist
        self._veh_odom_gt.header.stamp = rospy.Time.now()
        self._odom_pub.publish(self._veh_odom_gt)