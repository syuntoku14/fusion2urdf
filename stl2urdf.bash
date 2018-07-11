#!/usr/bin/env/bash
# Change the ascii stl to binary stl for creating urdf file.
# Put all stl files that you want to convert to binary into "mm_stl". 
# Then, run this script.

for filename in ./mm_stl/*.stl; do python mm2m_stl.py $filename; done
for filename in ./mm_stl/*_m.stl; do ruby convertSTL.rb $filename; done

mkdir m_stl bin_stl

rm *_m_m.stl
mv ./mm_stl/*_m.stl ./m_stl
mv ./mm_stl/*-binary.stl ./bin_stl

python ./joint_info/joints2urdf.py