#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np

#robot parmeters
L1 = 0.3
L2 = 0.3
#x is the variable distance of the second arm
x  = 0.1 
L2max = 0.5
L2min = 0.25


def setL(L20,thetad,x0,y0,xd,yd):
    global L2max, L2lin
    ##calculate initial laser position
    x0_laser = L20*np.cos(thetad) + x0
    y0_laser = L20*np.sin(thetad) + y0
    ##calculate L value
    L2_add = np.sqrt((xd-x0_laser)**2 + (yd-y0_laser)**2 )
    L2_fin = max(min(L2max,(L20 + L2_add)),L2min) 
    return L2_fin,x0_laser,y0_laser


def setTheta(xd,yd,x0,y0):
    ##calculate new theta
    theta_fin = np.arctan((yd-y0)/(xd-x0))
    return theta_fin
    
    
if __name__ == "__main__":
    #initial position arm1
    x0 = 0
    y0 = 0
    L20 = 0.25 
    theta0 = 0
    
    #desired position
    xd = 0.3
    yd = 0.3

    #calculate nex theta value
    thetad = setTheta(xd,yd,x0,y0)
    
    #calculate nex L value
    L2_fin,x0_laser,y0_laser = setL(L20,thetad,x0,y0,xd,yd)
    
    #calculate finaL laser position
    x_laser = (L2_fin*np.cos(thetad))
    y_laser =  (L2_fin*np.sin(thetad))
    

    print ("thetad = ",thetad)
    print ("L2 = ",L2_fin)
    print (x_laser,y_laser)

    if(x_laser != xd or y_laser != yd):
        print("Move robot!")

    print ("End ...")


