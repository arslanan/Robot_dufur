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
    
    if contours == []:
        return False,False
    
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

img = cv2.imread('cylindre.png')[:50,:50]
#cv2.imshow('coucou',img)
detect(img)