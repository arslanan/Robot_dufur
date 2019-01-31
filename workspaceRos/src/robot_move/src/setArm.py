#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import numpy as np

L2min= -0.2
L2max = 0 
def setL(C_arm,thetad,x0,y0,xd,yd):
    """
     setL calcule la nouvelle valeur du bras 
     selon la position désirée
    """
    global L2max, L2min
    #calcule longueur totale du bras dans le conditions actuelles
    longueur_arm = 0.4 + C_arm 
    #calcule la position du laser à partir de la
    #position initiale du laser et du theta désiré.
    x0_laser =  x0  + longueur_arm*np.cos(thetad) 
    y0_laser =  y0  + longueur_arm*np.sin(thetad) 
    #calcule la distance entre les points en question
    C_arm = np.sqrt((xd-x0_laser)**2 + (yd-y0_laser)**2 )
    #limite C_arm selon les conditions physiques
    C_arm = max(min(L2max,(C_arm)),L2min) 
    return C_arm


def setTheta(xd,x0, C_arm):
    """
     setTheta calcule la nouvelle valeur de theta 
     selon la position désirée
    """
    #calcule longueur totale du bras dans le conditions actuelles
    longueur_arm = 0.4 + C_arm
    #calcule taille du pixel
    taille_pixel = 0.038/400
    #calcule theta
    theta_fin = np.arcsin((xd-x0)* taille_pixel/longueur_arm)
    return theta_fin
