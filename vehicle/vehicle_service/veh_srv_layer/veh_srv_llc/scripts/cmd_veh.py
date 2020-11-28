#! /usr/bin/env python

from veh_srv_ros_types.msg import VehBrake, VehCfg, VehGear, VehSteering, VehThrottle, AudiCmd

class CmdVeh:
    def __init__(self, vehNs, vehStrLen=1, vehId=0, vehName='None'):
        self._vehNs = vehNs
        self._vehStrLen = vehStrLen
        self._vehId = vehId
        self._vehName = vehName

    def init_ros(self):
        self._throttle_obj.init_ros()
        self._brake_obj.init_ros()
        self._steering_obj.init_ros()
        self._gear_obj.init_ros()

    def gen_actuators_props(self):
        self._throttle_obj = self.gen_throttle_obj()
        self._brake_obj = self.gen_brake_obj()
        self._steering_obj = self.gen_steering_obj()
        self._gear_obj = self.gen_gear_obj()

    def init(self):
        self.gen_actuators_props()
        self.init_ros()

    # def pub_all(self, throttle_obj, brake_obj, steering_obj, gear_obj):

def init_veh_cmd(vehNs, vehStrLen=1, vehId=0, vehName='None'):
    cmd_veh_obj = CmdVeh(vehNs, vehStrLen, vehId, vehName)
    cmd_veh_obj.init()