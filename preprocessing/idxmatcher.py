__author__ = 'Shuo Yao'

import cv2
import os

def getName(num):
    strRes = ''

    n = len(str(num))
    for i in range(0, 5-n):
        strRes += '0'
    strRes += str(num)
    return strRes

def modify_img_name():
    img_list = []
    for i in range(1228):
        img_list.append(getName(i+1))
    f = open('C:\\Users\\shuoy\\Desktop\\test_video\\new_camera\\new_calib\\seq.txt', 'r')
    idx_list = []
    data = f.readlines()
    for idx in data:
        idx_list.append(idx.strip())
    for i in range(1228):
        folder = 'C:\\Users\\shuoy\\Desktop\\test_video\\new_camera\\small_calib\\small_calib\\cam1\\'
        fname = folder + img_list[i] + '.png'
        os.rename(fname, folder + idx_list[i] + '.png')

if __name__ == '__main__':
    modify_img_name()