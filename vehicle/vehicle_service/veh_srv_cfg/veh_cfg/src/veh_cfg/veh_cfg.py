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
        self._cmd_props_dict            = self.loc_gen_cmd_name()
        self._throt_node_name           = self._cmd_props_dict['throt']['node_name']
        self._throt_topic_in_name       = self._cmd_props_dict['throt']['topic_in']
        self._throt_topic_out_name      = self._cmd_props_dict['throt']['topic_out']
        self._throt_pub_rate            = self._cmd_props_dict['throt']['pub_rate']
        self._brake_node_name           = self._cmd_props_dict['brake']['node_name']
        self._brake_topic_in_name       = self._cmd_props_dict['brake']['topic_in']
        self._brake_topic_out_name      = self._cmd_props_dict['brake']['topic_out']
        self._brake_pub_rate            = self._cmd_props_dict['brake']['pub_rate']
        self._steer_node_name           = self._cmd_props_dict['steer']['node_name']
        self._steer_topic_in_name       = self._cmd_props_dict['steer']['topic_in']
        self._steer_topic_out_name      = self._cmd_props_dict['steer']['topic_out']
        self._steer_pub_rate            = self._cmd_props_dict['steer']['pub_rate']
        self._gear_node_name            = self._cmd_props_dict['gear']['node_name']
        self._gear_topic_in_name        = self._cmd_props_dict['gear']['topic_in']
        self._gear_topic_out_name       = self._cmd_props_dict['gear']['topic_out']
        self._gear_pub_rate             = self._cmd_props_dict['gear']['pub_rate']
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
        elif self._vehStrLen != 1:
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
            name = self._vehNs + '/veh_cfg'
        else:
            name = None
        return name

    def loc_gen_cmd_name(self):
        throt_node_name = self._vehNs + '_veh_throttle'
        brake_node_name = self._vehNs + '_veh_brake'
        steer_node_name = self._vehNs + '_veh_steering'
        gear_node_name = self._vehNs + '_veh_gear'
        throt_topic_in = self._vehNs + '/veh_throttle'
        brake_topic_in = self._vehNs + '/veh_brake'
        steer_topic_in = self._vehNs + '/veh_steering'
        gear_topic_in = self._vehNs + '/veh_gear'
        throt_topic_out = self._vehNs + '/throttle_cmd'
        brake_topic_out = self._vehNs + '/brake_cmd'
        steer_topic_out = self._vehNs + '/steering_cmd'
        gear_topic_out = self._vehNs + '/gear_cmd'
        throt_pub_rate = 100
        brake_pub_rate = 100
        steer_pub_rate = 100
        gear_pub_rate = 20
        throt_dict = {'node_name': throt_node_name, 'topic_in': throt_topic_in, 'topic_out': throt_topic_out, 'pub_rate': throt_pub_rate}
        brake_dict = {'node_name': brake_node_name, 'topic_in': brake_topic_in, 'topic_out': brake_topic_out, 'pub_rate': brake_pub_rate}
        steer_dict = {'node_name': steer_node_name, 'topic_in': steer_topic_in, 'topic_out': steer_topic_out, 'pub_rate': steer_pub_rate}
        gear_dict = {'node_name': gear_node_name, 'topic_in': gear_topic_in, 'topic_out': gear_topic_out, 'pub_rate': gear_pub_rate}
        cmd_dict = {'throt': throt_dict, 'brake': brake_dict, 'steer': steer_dict, 'gear': gear_dict}
        return cmd_dict

# class VehPhySrvCfg:

# class VehSimSrvCfg:

# class DynVehSrvCfg:

# class StrVehSrvCfg: