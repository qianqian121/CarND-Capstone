import cv2
import numpy as np
import math

# Functions to detect RED lights in real life
# Cropped and found pictures are included in this repo

ref_x = -0.837
ref_y = 15.79
ref_x_last = 8.95
ref_y_last = 18.03

def dist_to_ref(b_x, b_y):
    return math.sqrt((ref_x - b_x) ** 2 + (ref_y - b_y) ** 2)

def is_interesting(b_x,b_y):
    return ref_x<b_x<ref_x_last and ref_y<b_y<ref_y_last

def crop_variables(dist):
    rect = 200 + 10 * dist
    left_corner_x = 647 - 8 * dist #14.4
    left_corner_y = 412 - 4.3 * dist
    return (int(rect), int(left_corner_x), int(left_corner_y))

def glow_windows(img):
    #     print(img.shape)
    height = img.shape[0]
    width = img.shape[1]
    windows = [0]
    for i in range(1, 5):
        windows.append(int(i * width / 4.0))
    # print(width, height, windows)
    results = []
    for z in range(4):
        glows = 0  # high intensity color
        for i in range(height):
            for j in range(windows[z], windows[z + 1]):
                if (float(img[i][j][1]) + float(img[i][j][2])) / 2 > 230.0:
                    glows += 1
        results.append(glows)
    results.sort()
    if results[-2] > 150:  # if there are 2 glowing windows, probably we're talking about a bright image
        #         print('bright image')
        return 0

    return max(results)

# img = cv2.imread(file_name)
# roi = img[left_y:left_y+int(0.2*rec), left_x:left_x+int(1.2*rec)] # im[y1:y2, x1:x2] 1-> upper left corner 2 -> bottom right corner


