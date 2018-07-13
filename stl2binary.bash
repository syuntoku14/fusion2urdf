#!/usr/bin/env/bash

# input your robot name
# example: '''bash stl2binary.bash my_robot'''

for filename in ./$1/mm_stl/*.stl; do python mm2m_stl.py $filename; done
for filename in ./$1/mm_stl/*_m.stl; do ruby convertSTL.rb $filename; done

mkdir ./$1/m_stl ./$1/bin_stl

mv ./$1/mm_stl/*_m.stl ./$1/m_stl
mv ./$1/mm_stl/*-binary.stl ./$1/bin_stl
