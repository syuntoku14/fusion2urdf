#!/usr/bin/env python
# -*- coding: utf-8 -*-

from urdf_tools import *

def main():
    mm2m(joints_dict)  # convert milimeter to meter
    link_dict = {}
    repo = 'rockerbogie/For_URDF/bin_stl/'        
    file_name = 'test.urdf'
    robot_name = 'robot'

    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}">\n'.format(robot_name))

    link_gen(joints_dict, repo, link_dict, file_name)
    joint_gen(joints_dict, repo, link_dict, file_name)

    with open(file_name, mode='a') as f:
        f.write('</robot>')

if __name__ == '__main__':
    main()