#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 23 14:47:18 2019

@author: anto
"""
import numpy as np

L2min= -0.2
L2max = 0 
def setL(L20,thetad,x0,y0,xd,yd):
    global L2max, L2min
    ##calculate initial laser position
    Li = 0.4 + L20 
    x0_laser =  x0  + Li*np.cos(thetad) 
    y0_laser =  y0  + Li*np.sin(thetad) 
    ##calculate L value
    L2_add = np.sqrt((xd-x0_laser)**2 + (yd-y0_laser)**2 )
    L2_fin = max(min(L2max,(L2_add)),L2min) 
    return L2_fin,x0_laser,y0_laser


def setTheta(xd,yd,x0,y0):
    ##calculate new theta
    theta_fin = np.arctan((yd-y0)/(xd-x0))
    return theta_fin
