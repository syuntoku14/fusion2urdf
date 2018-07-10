#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
 [summary]
 Create xacro variable file fron joint information.

[description]
 <example>

 input: sample.txt(This has joint information of Fusion 360 CAD data)

 ```(sample.txt)
    leg_f_l -99.999 -127.661 -22.845 
    leg_b_r -68.00 34.40 33.80 
    wheel_f_r -136.999 -212.358 -95.775
 ```

 return: 'python joint_xacro.py sample.txt' will create .xacro file:

 ```(sample_m.urdf.xacro)
    <xacro:property name="leg_f_l_x" value=-0.099999 />
    <xacro:property name="leg_f_l_y" value=-0.127661 />
    <xacro:property name="leg_f_l_z" value=-0.022845 />
    <xacro:property name="leg_b_r_x" value=-0.068 />
    <xacro:property name="leg_b_r_y" value=0.0344 />
    <xacro:property name="leg_b_r_z" value=0.0338 />
    <xacro:property name="wheel_f_r_x" value=-0.136999 />
    <xacro:property name="wheel_f_r_y" value=-0.212358 />
    <xacro:property name="wheel_f_r_z" value=-0.095775 />
 ```

"""


import sys

def mm_to_m(mm):
    if mm != 'offset':
        m = float(mm) / 1e3
        m = '{:.6}'.format(m)
        return m


def xacro_val_gen(axis, name, value):
    txt = '<xacro:property name="'
    txt = txt + name + '_'+ axis + '" value='
    txt = txt + value + ' />'
    return txt


def main():
    assert sys.argv[1][-4:] == '.txt', 'Please input .txt file'
    file_name = sys.argv[1][:-4]  # file name without .txt
    path_r = file_name + '.txt'
    new_stl = []

    # load millimeter .txt file 
    with open(path_r) as stl:
        for line in stl:
            moji = list(line.split())
            new_stl.append(moji)
        
    path_w = file_name + '_m.urdf.xacro'

    with open(path_w, mode='w') as n_stl:

        name = None
        axis = ['x', 'y', 'z']

        for line in new_stl:
            if len(line) == 0:
                continue
            
            name = line[0]
            xyz = []

            for mm in line[1:]:
                m = mm_to_m(mm)
                if m != None :
                    xyz.append(m)

            for i, ax in enumerate(axis):
                n_stl.write(xacro_val_gen(ax, name, xyz[i]))
                n_stl.write('\n')

if __name__ == '__main__':
    main()
