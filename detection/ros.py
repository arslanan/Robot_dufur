#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 10 12:54:53 2018

@author: antony
"""

NodePictureName1 = "rrbot/camera1/camera_info"
NodePictureName2 = "rrbot/camera2/camera_info"
NodePicture1 = "rrbot/camera1/image_raw/compressed"
NodePicture2 = "rrbot/camera2/image_raw/compressed"
NodeCommande = "/cmd_vel"

import os
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from getAngleHauteur import getAngle
from detection_color import detect
import numpy as np
import cv2

ANGLE_MAX = 10
ANGLE_MIN = -10
EPSILON_HAUTEUR = 10 # pixels

# Calibration
#d1 =   --> h1 = ?
#d2 =   --> h2 = ?
#h=f(d)=ad+b
#a=(h2-h1)/(d2-d1)
#b = h1-a*d1

class data_getting():
    
    def __init__(self):
		
#		print('Init the variables')
        self.img1 = None
        self.img2 = None
        self.matrice1 = None
        self.matrice2 = None
        self.angle = None
        self.hauteur = None
        self.cx = None
        self.cy = None
        self.pub = rospy.Publisher(NodeCommande, Twist, queue_size=10)
        self.consigne = None
#		print(" Init the subscribers ")
        self.listener_mat1 = rospy.Subscriber(NodePictureName1, CameraInfo, self.callback_matrice1)	
        self.listener_mat2 = rospy.Subscriber(NodePictureName2, CameraInfo, self.callback_matrice2)
        self.listener_img1 = rospy.Subscriber(NodePicture1, CompressedImage, self.callback_img1)
        self.listener_img2 = rospy.Subscriber(NodePicture2, CompressedImage, self.callback_img2)
        self.listener_img2 = rospy.Subscriber(NodeCommande, Twist, self.callback_cmd)
    

    ## Callback pour les suscribers    
    def callback_matrice1(self,data):
        rospy.loginfo("I heard %s",data.data)
        self.matrice1 = data.P
        
    def callback_matrice2(self,data):
        rospy.loginfo("I heard %s",data.data)
        self.matrice2 = data.P	
        
    def callback_pic1(self,data):
        rawpic1 = data.data
        np_arr = np.fromstring(rawpic1, np.uint8)
        self.img1 =  cv2.imdecode(np_arr, cv2.IMREAD_COLOR)@self.matrice1
        
    def callback_pic2(self,data):
        rawpic2 = data.data
        np_arr = np.fromstring(rawpic2, np.uint8)
        self.img2 =  cv2.imdecode(np_arr, cv2.IMREAD_COLOR)@self.matrice2
    
    def callback_cmd(self,data):
        self.consigne = data
    
    
    # Fonction principale a appeler en boucle 
    def control(self):
        """
        
        Detecte la plante  --> il faudra ajouter si rien est detecté, bouger aléatoirement  
        La place en son centre
        S'avance vers elle 
        
        Quand il est bien placé apelle la fonction pour peindre  
        
        """
        
        
        
        if detect(self.img1)[0]!=False:
        
            self.cx,self.cy = detect(self.img1)
            self.angle, self.hauteur = getAngle(self.img1,self.cx,self.cy)
            # Regler angle
            if self.angle >= ANGLE_MAX:
                self.consigne.angular.z = 1
                self.pub.publish(self.consigne)
            if self.angle <= ANGLE_MIN:
                self.consigne.angular.z = -1
                self.pub.publish(self.consigne)
            else :
                # regle distance
                if self.hauteur >= EPSILON_HAUTEUR:
                    #avance
                    self.consigne.linear.x = 0.2
                if self.hauteur <= -EPSILON_HAUTEUR:
                    #recule
                    self.consigne.linear.x = -0.2
                else:
                    pass #peindre
                                    
        else : # On ne detecte pas de plante, il fausdra bouger aléatoirement
            pass

		
def main():
	rospy.init_node('anything', anonymous=False)
	data = data_getting()
	rate = rospy.Rate(4)
	
	while not rospy.is_shutdown() :
		data.control()
		rate.sleep()	
		
	
	
if __name__ == '__main__':
	main()
				

	
