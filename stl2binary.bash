#!/usr/bin/env/bash

for filename in ./mm_stl/*.stl; do python mm2m_stl.py $filename; done
for filename in ./mm_stl/*_m.stl; do ruby convertSTL.rb $filename; done

mkdir m_stl bin_stl

rm *_m_m.stl
mv ./mm_stl/*_m.stl ./m_stl
mv ./mm_stl/*-binary.stl ./bin_stl
