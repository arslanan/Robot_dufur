#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2

""" Fonction permettant de trouver le centre d'une mauvaise herbe sur une image"""

def detect(img):
    """
     La fonction detect reçois une images d'une des caméras et
     elle trouve les contours s'il y a des herbes sur l'image.
     Elle retourne les coordonées du centre du contour de l'herbe 
     avec le plus grand contour.
    """
    
    print("begin function")
    # Transformation en HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # Seuils de saturation
    lower_green = np.array([50,100,100])
    upper_green = np.array([70,255,255])
    # Saturation de l'image
    imgSeg = cv2.inRange(imgHSV,lower_green,upper_green)
    # Trouver les contours
    _, contours, hierarchy = cv2.findContours(imgSeg,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Si il ne trouve pas de contours sur l'image
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
    #Condition pour éviter une division par zéro
    if M['m00']:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    else:
        cx = int(M['m10'])
        cy = int(M['m01'])
     
    return cx,cy



