#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob

# first = cv2.imread("/home/horimoto/trans_test/cartesian_1529297997_16.jpg", 0)
# first = np.float32(first)

# path_list = glob.glob("/home/horimoto/trans_test/*.jpg")
# path_list.sort()

# save_path = ("/home/horimoto/trans_test/transed/")

# rows, cols = first.shape
# m_def = np.float32([[1, 0, 0 + cols / 2], [0, 1, 0 + rows]])

# for path in path_list:
#     img = cv2.imread(path, 0)
#     img = np.float32(img)
#     (dx, dy), _ = cv2.phaseCorrelate(img, first)
#     m = np.float32([[1, 0, dx + cols / 2], [0, 1, dy + rows]])
#     dst = cv2.warpAffine(img, m, (cols * 2, rows * 2))

#     filename = path.split("/")[-1]
#     print filename
#     cv2.imwrite(save_path + filename, dst)

path_list = glob.glob("/home/horimoto/trans_test/*.jpg")
path_list.sort()

save_path = ("/home/horimoto/trans_test/transed/")

prev_path = path_list[0]
prev = cv2.imread(prev_path, 0)
path_list.pop(0)
rows, cols = prev.shape
m = np.float32([[1, 0, 0 + cols / 2], [0, 1, 0 + rows]])

for path in path_list:
    prev = np.float32(cv2.imread(prev_path, 0))
    img = np.float32(cv2.imread(path, 0))
    (dx, dy), _ = cv2.phaseCorrelate(img, prev)
    m[0, 2] += dx
    m[1, 2] += dy
    dst = cv2.warpAffine(img, m, (cols * 2, rows * 2))
    filename = path.split("/")[-1]
    print filename
    cv2.imwrite(save_path + filename, dst)

    prev_path = path
    