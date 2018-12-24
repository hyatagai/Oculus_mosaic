#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt

def to_np_mat(inp):
    r, c, ch = inp.shape
    ret = np.zeros((r, c)) + 0j
    for i in range(r):
        for j in range(c):
            ret[i][j] = inp[i][j][0] + 1j * inp[i][j][1]
    return ret

def to_cv_mat(inp):
    r, c = inp.shape
    ret = np.zeros((r, c, 2))
    for i in range(r):
        for j in range(c):
            ret[i][j][0] = np.real(inp[i][j])
            ret[i][j][1] = np.imag(inp[i][j])
    return ret


img = cv2.imread('/home/horimoto/img1.jpg', 0)
# cv2.imshow('image 1', img)

img2 = cv2.imread('/home/horimoto/img2.jpg', 0)
# cv2.imshow("image 2", img2)

img = cv2.GaussianBlur(img, (5, 5), 0)
img2 = cv2.GaussianBlur(img2, (5, 5), 0)


img = np.float32(img)
img2 = np.float32(img2)
(dx, dy), _ = cv2.phaseCorrelate(img, img2)
print dx, dy

rows, cols = img.shape
m_def = np.float32([[1, 0, 0 + cols], [0, 1, 0 + rows]])
m = np.float32([[1, 0, -dx + cols], [0, 1, -dy + rows]])
dst = cv2.warpAffine(img, m_def, (cols * 2, rows * 2))
dst2 = cv2.warpAffine(img2, m, (cols * 2, rows * 2))

cv2.imwrite("/home/horimoto/res1.bmp", dst)
cv2.imwrite("/home/horimoto/res2.bmp", dst2)


# plt.subplot(121), plt.imshow(img2, cmap="gray")
# plt.subplot(122), plt.imshow(dst, cmap="gray")
# plt.show()

# dft1 = cv2.dft(np.float32(img), flags = cv2.DFT_COMPLEX_OUTPUT)
# dft2 = cv2.dft(np.float32(img2), flags = cv2.DFT_COMPLEX_OUTPUT)
# dft1_comp = to_np_mat(dft1)
# dft2_comp = to_np_mat(dft2)

# r = dft1_comp * np.conj(dft2_comp)
# ret = r / np.abs(r)

# ret_cv = to_cv_mat(ret)
# cor = cv2.idft(ret_cv)
# cor = cv2.magnitude(cor[:, :, 0], cor[:, :, 1])
# res = np.argmax(cor)
# r, c = cor.shape
# print res / r, res % r



# plt.subplot(131), plt.imshow(img, cmap="gray")
# plt.subplot(132), plt.imshow(img2, cmap="gray")
# plt.subplot(133), plt.imshow(cor, cmap="gray")

# plt.show()


# conj_dft2 = dft2
# conj_dft2[:, :, 1] *= -1
# correlation = dft1 * dft2


# plt.subplot(121), plt.imshow(img, cmap="gray")
# plt.title("Input image"), plt.xticks([]), plt.yticks([])
# plt.subplot(122), plt.imshow(img_back, cmap="gray")
# plt.title("Magnitude Spectrum"), plt.xticks([]), plt.yticks([])
# plt.show()

# plt.subplot(121), plt.imshow(img, cmap="gray")
# plt.title("hoge"), plt.xticks([]), plt.yticks([])
# plt.subplot(122), plt.imshow(mag, cmap="gray")
# plt.title("mag"), plt.xticks([]), plt.yticks([])
# plt.show()
