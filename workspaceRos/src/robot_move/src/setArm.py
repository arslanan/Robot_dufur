#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 23 14:47:18 2019

@author: anto
"""
import numpy as np

L2min= -0.2
L2max = 0 
def setL(C_arm,thetad,x0,y0,xd,yd):
    global L2max, L2min
    ##calculate initial laser position
    longueur_arm = 0.4 + C_arm 
    x0_laser =  x0  + longueur_arm*np.cos(thetad) 
    y0_laser =  y0  + longueur_arm*np.sin(thetad) 
    ##calculate L value
    C_arm = np.sqrt((xd-x0_laser)**2 + (yd-y0_laser)**2 )
    C_arm = max(min(L2max,(C_arm)),L2min) 
    return C_arm,x0_laser,y0_laser


def setTheta(xd,yd,x0,y0, C_arm):
    longueur_arm = 0.4 + C_arm
    ##calculate new theta
    taille_pixel = 0.038/400
    theta_fin = np.arcsin((xd-x0)* taille_pixel/longueur_arm)
    return theta_fin
