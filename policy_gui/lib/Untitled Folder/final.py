import cv2
import numpy as np
from skimage.measure import compare_ssim as ssim
	
x0 = cv2.imread("testmap.png")

x1 = cv2.imread("parking.png")
x2 = cv2.imread("tleft.png")
x3 = cv2.imread("tright.png")
x4 = cv2.imread("tnormal.png")
x5 = cv2.imread("x.png")

s1 = ssim(x0, x1,multichannel = True) 
s2 = ssim(x0, x2,multichannel = True)
s3 = ssim(x0, x3,multichannel = True)
s4 = ssim(x0, x4,multichannel = True)
s5 = ssim(x0, x5,multichannel = True)

sm = min(s2,s3,s4,s5)

print("   ")

if (sm == s2) and s1 != 1 and s3 != 1 and s4 != 1 and s5 !=1:
        print("Image is T-Left")
if (sm == s3) and s1 != 1 and s2 != 1 and s4 != 1 and s5 !=1:
        print("Image is T-Right")
if sm == s4 and s1 != 1 and s3 != 1 and s2 != 1 and s5 !=1:
        print("Image is T-Normal")
if sm == s5 and s1 != 1 and s3 != 1 and s4 != 1 and s2 !=1:
        print("Image is Cross-X")

if (s1 == 1):
        print("Image is Parking")		
if (s2 == 1):
        print("Image is T-Left")
if (s3 == 1):
        print("Image is T-Right")
if (s4 == 1):
        print("Image is T-Normal")
if (s5 == 1):
        print("Image is Cross-X")