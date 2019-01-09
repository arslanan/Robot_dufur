import sys
import time
import cv2
from naoqi import ALProxy
import numpy as np
#import Image
import random
import math
from PID import PID
pidx = PID(1, 0.1, 0.05, setpoint=1)
pidy = PID(1, 0.1, 0.05, setpoint=1)

debug=False

def getAngle(cameraImage,cx,cy):
    # Voir ce que voit le robot
    cv2.namedWindow("proc")
    cv2.resizeWindow("proc",imageWidth,imageHeight)
    cv2.moveWindow("proc",0,0)   

    [l,L,_] = cameraImage.shape
    errx = cx-l/2
    erry = cy-L/2
          
    angle = math.atan2((l/2-cx),(L/2-cy))
    hauteur = cy - L
         
    return angle, hauteur