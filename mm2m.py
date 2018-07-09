#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
 [summary]
 Convert the unit of STL file to meter from millimeter.

[description]
 Run the following code to generate meter stl file.
 
 "python mm_to_m.py sample.stl"
 
 This create sample_m.stl file its unit is meter.

 If you want to change some stl files at onece, run the following code.
 "for filename in *.stl; do python3 mm_to_m.py $filename; done"
"""


import sys

def mm_to_m(mm):
    if mm != 'vertex':
        m = float(mm) / 1e3
        m = '{:.6e}'.format(m)
        idx = '0' + m[-2:]
        return m[:11] + idx

def main():
    assert sys.argv[1][-4:] == '.stl', 'stlファイルを入力してください'
    file_name = sys.argv[1][:-4]  # file名の拡張子を除いた部分
    path_r = file_name + '.stl'
    new_stl = []

    # mm単位のstlファイルを読み込む
    with open(path_r) as stl:
        for line in stl:
            moji = list(line.split())
            new_stl.append(moji)
        
    path_w = file_name + '_m.stl'

    with open(path_w, mode='w') as n_stl:
        for line in new_stl:
            # vertex 以外の文字列はそのまま
            if len(line) == 0:
                continue

            if line[0] != 'vertex':
                n_stl.write(' '.join(line))
                n_stl.write('\n')
            # vertexは mm から m に変換する
            else:
                temp = ['vertex']
                for mm in line:
                    m = mm_to_m(mm)
                    if m != None :
                        temp.append(m)
                n_stl.write(' '.join(temp))
                n_stl.write('\n')

if __name__ == '__main__':
    main()
