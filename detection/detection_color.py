import numpy as np
import cv2

""" Fonction permettant de trouver le centre d'une mauvaise herbe sur une image"""

def detect(img):
    # Transformation en HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # Seuils de saturation
    lower_green = np.array([50,100,100])
    upper_green = np.array([70,255,255])
    # Saturation de l'image
    imgSeg = cv2.inRange(imgHSV,lower_green,upper_green)
    # Trouver les contours
    contours, hierarchy = cv2.findContours(imgSeg,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    return cx,cy
    
if __name__ == "__main__":
    # read image from file
    img = cv2.imread("cylindre.png")
    # get dimensions
    imageHeight, imageWidth, imageChannels = img.shape
    # create a window
    cv2.namedWindow("Image")
    # resize the window to match image dimensions
    cv2.resizeWindow("Image",imageWidth,imageHeight)
    # move image to upper-left corner
    cv2.moveWindow("Image",0,0)
     
    # Transformation en HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
    lower_green = np.array([50,100,100])
    upper_green = np.array([70,255,255])
     
    #H = imgHSV[:,:,0]
    imgSeg = cv2.inRange(imgHSV,lower_green,upper_green)
     
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
    #img_contour[cy][cx] = [0,255,0]
    cv2.circle(img_contour,(cx,cy),5,color=(0,0,255))
    cv2.imshow('Image contours',img_contour)
    cv2.waitKey(0)
