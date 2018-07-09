#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

def mm_to_m(mm):
    if mm != 'offset':
        m = float(mm) / 1e3
        m = '{:.6}'.format(m)
        return m

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
            if len(line) == 0:
                continue

            # offset 以外の文字列はそのまま
            if line[0] != 'offset':
                n_stl.write(' '.join(line))
                n_stl.write('\n')
            # offsetは mm から m に変換する
            else:
                temp = ['offset']
                for mm in line:
                    m = mm_to_m(mm)
                    if m != None :
                        temp.append(m)
                n_stl.write(' '.join(temp))
                n_stl.write('\n')

if __name__ == '__main__':
    main()
