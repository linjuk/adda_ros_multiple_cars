import numpy as np
import cv2

img1 = cv2.imread('testmap5.png')
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
ret1,thresh1 = cv2.threshold(gray1,127,255,1)
im1, contours1,h1 = cv2.findContours(thresh1,1,2)
x=0


for cnt1 in contours1:
    approx1 = cv2.approxPolyDP(cnt1,0.01*cv2.arcLength(cnt1,True),True)

    if len(approx1)==4:
        x = x +1
        cv2.drawContours(img1,[cnt1],0,(0,0,255),-1)
		        
if x == 8:
    print("This Image is X-intersection")

elif x == 4:
    print("This Image is T-intersection")