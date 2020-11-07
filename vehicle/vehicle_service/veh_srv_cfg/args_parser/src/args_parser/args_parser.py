#! /usr/bin/env python

import sys

class ArgsParser:
    def vehSrvCfg_parse_args(self):
        args = None
        args_list = list(sys.argv)
        if len(args_list) > 1:
            args_list.pop(0)
            args = args_list
        return args