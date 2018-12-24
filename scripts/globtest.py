#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import copy

def register_FLS_image(polar_image1, cartesian_image1, polar_image2, cartesian_image2):
    polar_image1 = copy.deepcopy(np.float32(polar_image1))
    polar_image2 = copy.deepcopy(np.float32(polar_image2))
    cartesian_image1 = copy.deepcopy(np.float32(cartesian_image1))
    cartesian_image2 = copy.deepcopy(np.float32(cartesian_image2))
    polar_image1 = cv2.GaussianBlur(polar_image1, (3, 3), 0)
    polar_image2 = cv2.GaussianBlur(polar_image2, (3, 3), 0)
    cartesian_image1 = cv2.GaussianBlur(cartesian_image1, (3, 3), 0)
    cartesian_image2 = cv2.GaussianBlur(cartesian_image2, (3, 3), 0)

    (polar_dx, _), _ = cv2.phaseCorrelate(polar_image1, polar_image2)
    theta = polar_dx / 10.    #[degree]

    rows, cols = cartesian_image1.shape
    m_def = np.float32([[1, 0, cols / 2], [0, 1, 0]])   #回転した時に情報が消えないように範囲を広げる
    cartesian_image1 = cv2.warpAffine(cartesian_image1, m_def, (cols*2, rows*2))
    cartesian_image2 = cv2.warpAffine(cartesian_image2, m_def, (cols*2, rows*2))
    rotate_matrix = cv2.getRotationMatrix2D((cols, rows), theta, 1)
    rotated_cartesian_image2 = cv2.warpAffine(cartesian_image2, rotate_matrix, (cols*2, rows*2))
    (dx, dy), _ = cv2.phaseCorrelate(cartesian_image1, rotated_cartesian_image2)
    
    return theta, dx, dy

if __name__ == "__main__":
    path_list = glob.glob("/home/umigame/seatec/multibeam_image/2018-10-04-14-58-00/cartesian_*.jpg")
    path_list.sort()
    save_path = ("/home/umigame/seatec/multibeam_image/2018-10-04-14-58-00/無題のフォルダー/")

    prev_path = path_list[0]
    prev = cv2.imread(prev_path, 0)
    path_list.pop(0)
    rows, cols = prev.shape
    offset_rows = rows * 3
    offset_cols = cols * 3
    nowtheta, nowx, nowy = -90., 0., 0.
    prev_img = cv2.imread(prev_path, 0)

    for path in path_list:
           
        prev_ctn = cv2.imread(prev_path, 0)
        prev_plr = cv2.imread(prev_path.replace("cartesian", "polar"), 0)
        img_ctn = cv2.imread(path, 0)
        img_plr = cv2.imread(path.replace("cartesian", "polar"), 0)
        print (prev_path, path)
    # if prev_ctn.shape[0] != 614:
        prev_ctn = cv2.resize(prev_ctn, (1114, 614))
    # if img_ctn.shape[0] != 614:
        img_ctn = cv2.resize(img_ctn, (1114, 614))
    # if prev_plr.shape[0] != 614:
        prev_plr = cv2.resize(prev_plr, (1301, 614))
    # if img_plr.shape[0] != 614:
        img_plr = cv2.resize(img_plr, (1301, 614))


        theta, dx, dy = register_FLS_image(prev_plr, prev_ctn, img_plr, img_ctn)
        nexttheta = nowtheta + theta
        nextx = nowx + dy * np.cos(np.deg2rad(nowtheta)) - dx * np.sin(np.deg2rad(nowtheta))
        nexty = nowy + dy * np.sin(np.deg2rad(nowtheta)) + dx * np.cos(np.deg2rad(nowtheta))

        rotate_shift_x = rows - cols / 2
        img_ctn_margined = cv2.warpAffine(img_ctn, np.float32([[1, 0, rotate_shift_x], [0, 1, 0]]), (cols*2, rows*2))
        rotate_matrix = cv2.getRotationMatrix2D((cols, rows), nexttheta + 90, 1)
        rotated_img_ctn = cv2.warpAffine(img_ctn, rotate_matrix, (cols*2, rows*2))
        translation_matrix = np.float32([[1, 0, nextx + (offset_cols - cols) / 2 - rotate_shift_x], [0, 1, nexty + offset_rows - rows * 1.5]])
        result_img= cv2.warpAffine(src=rotated_img_ctn, M=translation_matrix, dsize=(offset_cols, offset_rows), dst=prev_img, borderMode=cv2.BORDER_TRANSPARENT)
       

        filename = path.split("/")[-1]
        save_name = save_path + filename[10:21] + filename[21:].zfill(7)
        cv2.imwrite(save_name, result_img)
        prev_img=result_img

        prev_path = path
        nowtheta = nexttheta
        nowx = nextx
        nowy = nexty
  

    
    