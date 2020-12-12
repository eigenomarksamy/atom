#! /usr/bin/env python

import sys
import argparse

class ArgsParser:
    def __init__(self):
        self._parser = argparse.ArgumentParser(description='Simulated vehicle parser.')
        self._parser.add_argument('--ns', nargs="?", const='audibot', help='Vehicle namespace')
        self._parser.add_argument('--id', nargs="?", type=int, const=1, help='Vehicle ID')
        self._parser.add_argument('--vehiclename', nargs="?", const='None', help='Vehicle name')
        self._parser.add_argument('--override_ns', dest='override_ns', action='store_true', help='Flag to override namespace with vehicle name')
        self._parser.add_argument('--sim', dest='sim', action='store_true', help='Simulation-based ODOM GT publishing node')
        self._parser.add_argument('__name', help='Non-usable argument for ROS')
        self._parser.add_argument('__log', help='Non-usable argument for ROS')

    def get_args(self, args=sys.argv[1:]):
        self._arg_options, unknown = self._parser.parse_known_args()
        return self._arg_options

    def get_veh_launch_params(self, args=sys.argv[1:]):
        self._arg_options = self._parser.parse_args(args)
        ns = self._arg_options.ns
        id = self._arg_options.id
        name = self._arg_options.vehiclename
        odom_sim = self._arg_options.sim
        self._odom_dict = {'sim': odom_sim}
        self._veh_dict = {'ns': ns, 'id': id, 'name': name}
        self._args_dict = {'veh_params': self._veh_dict, 'odom_gt': self._odom_dict}
        return self._args_dict

def parse_args_manual():
    args = None
    args_list = list(sys.argv)
    if len(args_list) > 1:
        args_list.pop(0)
        args = args_list
    return args