#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
import sys
import time
import numpy as np
#import Image
import random
import math
import cv2

""" Fonction renvoyant l'angle et la différence de hauteur entre le centre de
la caméra et docn du robot et le centre de la mauvaise herbe """

debug=False

def getAngle(cameraImage,cx,cy):
    [l,L,_] = cameraImage.shape
    
    # Voir ce que voit le robot
    cv2.namedWindow("proc")
    cv2.resizeWindow("proc",l,L)
    cv2.moveWindow("proc",0,0)   

    errx = cx-l/2
    erry = cy-L/2
          
    angle = math.atan2((l/2-cx),(L/2-cy))
    hauteur = cy - L
         
    return angle, hauteur
