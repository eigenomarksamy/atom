#! /usr/bin/env python

class VehSrvCfg:
    def __init__(self, vehName, vehId, vehNs):
        self._vehName   = vehName
        self._vehId     = vehId
        self._vehNs     = vehNs
        self._gazebo_model_name = self._vehNs
        self._gazebo_frame_id   = '/' + self._vehNs + '/base_link'