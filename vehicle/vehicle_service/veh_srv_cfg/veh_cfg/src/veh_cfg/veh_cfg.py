#! /usr/bin/env python

class VehSrvComCfg:
    def __init__(self, vehNs, vehStrLen=1, vehName="None", vehId=0):
        self._vehNs                     = vehNs
        self._vehStrLen                 = vehStrLen
        self._vehName                   = vehName
        self._vehId                     = vehId
        self._gaz_model_name            = self._vehNs
        self._gaz_frame_id              = '/' + self._vehNs + '/base_link'
        self._odomgt_node_name          = self.loc_gen_odomgt_name('node')
        self._odomgt_topic_name         = self.loc_gen_odomgt_name('topic')
        self._odomgt_gaz_srv_name       = '/gazebo/get_model_state'
        self._cfg_node_name             = self.loc_gen_cfg_name('node')
        self._cfg_topic_name            = self.loc_gen_cfg_name('topic')
        self._pub_vehNs                 = self._vehNs
        self._pub_vehStrLen             = self._vehStrLen
        self._pub_vehName               = self._vehName
        self._pub_vehId                 = self._vehId
        self._pub_gazModelName          = self._gaz_model_name
        self._pub_gazFrameId            = self._gaz_frame_id
        self._pub_cfgNodeName           = self._cfg_node_name
        self._pub_cfgTopicName          = self._cfg_topic_name
        self._pub_odomgtNodeName        = self._odomgt_node_name
        self._pub_odomgtTopicName       = self._odomgt_topic_name
        self._pub_odomgtGazSrvName      = self._odomgt_gaz_srv_name

    def loc_gen_odomgt_name(self, req_type):
        if self._vehStrLen == 1:
            if req_type is 'node':
                name = self._vehNs + '_odom_gt'
            else:
                name = self._vehNs + '/odom/gt'
        elif self._vehStrLen == 1:
            if req_type is 'topic':
                name = self._vehNs + str(self._vehId) + '_odom_gt'
            else:
                name = self._vehNs + str(self._vehId) + '/odom/gt'
        else:
            name = None
        return name

    def loc_gen_cfg_name(self, req_type):
        if req_type is 'node':
            name = self._vehNs + '_veh_cfg'
        elif req_type is 'topic':
            name = self._vehNs + '/veh/cfg'
        else:
            name = None
        return name

# class VehPhySrvCfg:

# class VehSimSrvCfg:

# class DynVehSrvCfg:

# class StrVehSrvCfg: