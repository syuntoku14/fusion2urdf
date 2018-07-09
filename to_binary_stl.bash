#!/usr/bin/env/bash
# stlデータをURDFのmeshファイルとして使えるようにバイナリに変換します
# このディレクトリに変換したいSTLファイルを入れ、このスクリプトを実行してください

for filename in *.stl; do python3 mm2m.py $filename; done
for filename in *_m.stl; do ruby convertSTL.rb $filename; done

mkdir mm_stl m_stl bin_stl

mv *_m.stl ./m_stl
mv *-binary.stl ./bin_stl
mv *.stl ./mm_stl
