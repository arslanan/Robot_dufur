import numpy as np
import cv2
"""
OpenCV code to read and display an image
"""
# read image from file
img = cv2.imread("naoimg_001.png")
# get dimensions
imageHeight, imageWidth, imageChannels = img.shape
# create a window
cv2.namedWindow("Nao Image")
# resize the window to match image dimensions
cv2.resizeWindow("Nao Image",imageWidth,imageHeight)
# move image to upper-left corner
cv2.moveWindow("Nao Image",0,0)
# show the image
cv2.imshow("Nao Image",img) 


# Transformation en HSV
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
cv2.imshow('HSV',imgHSV)

lower_orange = np.array([0,100,100])
upper_orange = np.array([10,255,255])
#
#H = imgHSV[:,:,0]
imgSeg = cv2.inRange(imgHSV,lower_orange,upper_orange)
cv2.imshow('Sature',imgSeg)

#Trouve les contours
contours, hierarchy = cv2.findContours(imgSeg,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
img_contour = img
cv2.drawContours(img_contour, contours, -1, (255,255,255))

# Liste des tailles des contours
taille = []
for i in range(len(contours)):
    taille.append(cv2.contourArea(contours[i]))
    
taille = np.array(taille)

#Selection du plus grand contour
pose = np.argmax(taille)
#Barycentre du contour
M = cv2.moments(contours[pose])
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
img_contour[cy][cx] = [0,255,0]
cv2.imshow('Image contours',img_contour)
cv2.waitKey(0)
