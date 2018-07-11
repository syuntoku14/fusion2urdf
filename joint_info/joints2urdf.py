#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
 [summary]
 Generate urdf file from stl files automatically. Before run this code, write the joints data at joints.py.

 [description]
 Change the variables before you run this code.
 * repo: the repository in which the binary stl files are located
         (Make sure that the repository is in a ROS pakcage repository otherwise <mesh> doesn't work.)
 * file_name: output file name
 * robot_name: the name of the robot
"""

from urdf_tools import *

def main():
    mm2m(joints_dict)  # convert milimeter to meter
    link_dict = {}
    repo = 'package_name/repo_of_binary_stl/'        
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