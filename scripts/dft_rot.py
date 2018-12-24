#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob

def register_FLS_image(polar_image1, cartesian_image1, polar_image2, cartesian_image2):
    polar_image1 = np.float32(polar_image1)
    polar_image2 = np.float32(polar_image2)
    cartesian_image1 = np.float32(cartesian_image1)
    cartesian_image2 = np.float32(cartesian_image2)

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
    polar1 = "/home/horimoto/pol_rot1.jpg"
    polar2 = "/home/horimoto/pol_rot2.jpg"
    cart1 = "/home/horimoto/cart_rot1.jpg"
    cart2 = "/home/horimoto/cart_rot2.jpg"
    plr_im1 = cv2.imread(polar1, 0)
    plr_im2 = cv2.imread(polar2, 0)
    ctn_im1 = cv2.imread(cart1, 0)
    ctn_im2 = cv2.imread(cart2, 0)

    theta, dx, dy = register_FLS_image(plr_im1, ctn_im1, plr_im2, ctn_im2)
    print theta, dx, dy

    # cv2.imwrite("/home/horimoto/all_res1.jpg", im1)
    # cv2.imwrite("/home/horimoto/all_res2.jpg", im2)

