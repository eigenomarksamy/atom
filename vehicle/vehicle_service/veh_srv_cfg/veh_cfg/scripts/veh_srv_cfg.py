#! /usr/bin/env python

class VehSrvCfg:
    def __init__(self, vehNs, vehName=None, vehId=0):
        self._vehName   = vehName
        self._vehId     = vehId
        self._vehNs     = vehNs
        self._gaz_model_name = self._vehNs
        self._gaz_frame_id   = '/' + self._vehNs + '/base_link'

class VehSrvCfgOdomGt(VehSrvCfg):
    def __init__(self, vehNs, vehName=None, vehId=0):
        VehSrvCfg.__init__(self, vehNs, vehName, vehId)
        self._node_name = self._vehNs + str(self._vehId) + '_odom_gt'
        self._topic_name = self._vehNs + str(self._vehId) + '/odom/gt'
        self._gaz_srv_name = '/gazebo/get_model_state'