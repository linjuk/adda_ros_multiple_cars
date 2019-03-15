import numpy as np
import cv2

img1 = cv2.imread('testmap5.png')
img2 = cv2.imread('testmap6_0.png')
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
ret1,thresh1 = cv2.threshold(gray1,127,255,1)
ret2,thresh2 = cv2.threshold(gray2,127,255,1)
img1, contours1,h1 = cv2.findContours(thresh1,1,2)
img2, contours2,h2 = cv2.findContours(thresh2,1,2)
x=0
y=0

for cnt1 in contours1:
    approx1 = cv2.approxPolyDP(cnt1,0.01*cv2.arcLength(cnt1,True),True)

    if len(approx1)==4:
        x = x +1
        cv2.drawContours(img1,[cnt1],0,(0,0,255),-1)

for cnt2 in contours2:
    approx2 = cv2.approxPolyDP(cnt2,0.01*cv2.arcLength(cnt2,True),True)

    if len(approx2)==4:
        y = y +1
        cv2.drawContours(img2,[cnt2],0,(0,0,255),-1)

		
print("x = ",x)
print("y = ",y)
        
if x > y:
    cv2.waitKey(0)
    cv2.imwrite('X.png',img1)
    cv2.imwrite('T.png',img2)
    cv2.destroyAllWindows()
elif x < y:
    cv2.waitKey(0)
    cv2.imwrite('T.png',img1)
    cv2.imwrite('X.png',img2)
    cv2.destroyAllWindows()