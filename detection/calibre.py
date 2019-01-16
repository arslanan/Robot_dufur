#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  9 17:00:47 2019

@author: brugieju
"""



# =============================================================================
# Suscriber

def callback(data):
    rospy.loginfo("I heard %s",data.data)
     
def listener1():
    rospy.init_node('Listener_camera')
    cam1 = rospy.Subscriber("rrbot/camera1/camera_info", CameraInfo, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
def listener2():
    rospy.init_node('Listener_camera')
    cam2 = rospy.Subscriber("rrbot/camera2/camera_info", CameraInfo, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# =============================================================================
# Publisher
     
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('command')
r = rospy.Rate(10) # 10hz
 
while not rospy.is_shutdown():
    pub.publish()
    r.sleep()
# =============================================================================
