__author__ = 'Shuo Yao'

import cv2
import os
import time

def getName(num):
    strRes = ''

    n = len(str(num))
    for i in range(0, 5-n):
        strRes += '0'
    strRes += str(num)
    return strRes

def save_img():
    video_name = 'C:\\Users\\shuoy\\Desktop\\test_video\\extra\\st.mp4'
    folder_name = 'C:\\Users\\shuoy\\Desktop\\test_video\\extra\\st'
    folder_name2 = 'C:\\Users\\shuoy\\Desktop\\test_video\\extra\\st'
    txt_path = 'C:\\Users\\shuoy\\Desktop\\test_video\\extra\\st'
    os.makedirs(folder_name, exist_ok=True)
    vc = cv2.VideoCapture(video_name)
    c = 0
    rval = vc.isOpened()

    fps = vc.get(cv2.CAP_PROP_FPS)
    gap = 1/fps
    print(fps)
    nowTime = lambda: int(round(time.time() * 1000))

    file_object = open(txt_path + '/seq.txt', 'w')
    Ostr = ''

    while rval:
        current_time = time.time_ns()
        print(current_time)

        c = c + 1
        temp = c % 26
        rval, frame = vc.read()
        pic_path_1 = folder_name + '/'
        pic_path_2 = folder_name2 + '/'
        if rval:
            if temp == 0:
                tt = int(c/26)

                # percent by which the image is resized
                scale_percent = 100

                # calculate the 50 percent of original dimensions
                width = int(frame.shape[1] * scale_percent / 100)
                height = int(frame.shape[0] * scale_percent / 100)

                # dsize
                dsize = (width, height)

                # resize image
                output = cv2.resize(frame, dsize)

                # cv2.imwrite(pic_path_1 + str(current_time) + '.png', output)
                cv2.imwrite(pic_path_2 + getName(tt) + '.png', output)
                Ostr = Ostr + str(current_time) + '\n'
                #cv2.waitKey(1)
                time.sleep(gap)
        else:
            break

    vc.release()
    file_object.writelines(Ostr)
    file_object.close()
    print('save_success')
    print(folder_name)

if __name__ == '__main__':
    save_img()