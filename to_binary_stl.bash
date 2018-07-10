#!/usr/bin/env/bash
# Change the ascii stl to binary stl for creating urdf file.
# Put all stl files that you want to convert to binary into this repository. 
# Then, run this script.

for filename in *.stl; do python3 mm2m.py $filename; done
for filename in *_m.stl; do ruby convertSTL.rb $filename; done

mkdir mm_stl m_stl bin_stl

mv *_m.stl ./m_stl
mv *-binary.stl ./bin_stl
mv *.stl ./mm_stl

mkdir joint_info
python info2xacro.py info_joint.txt
mv info_joint.txt info_joint_m.urdf.xacro joint_info
