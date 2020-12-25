#!/usr/bin/env python

import cv2
from matplotlib import pyplot as plt
from vitarana_drone.msg import * 
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from pyzbar.pyzbar import decode


class detect_marker():
    def __init__(self):
        rospy.init_node('detect_marker_cascade') #Initialise rosnode
        self.img = np.empty([]) 
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.logo_cascade = cv2.CascadeClassifier('data/cascade.xml')
        rospy.Subscriber('/edrone/range_finder_bottom' , LaserScan , self.bottom)
        self.pub = rospy.Publisher('/edrone/coordinate_img', coordinates ,queue_size = 1)
        rospy.Subscriber('/marker_id',MarkerData, self.marker_id)
        self.Z_m = 0.0
        self.coordinates = coordinates()
        
        self.coordinates.x =0
        self.coordinates.y =0
        self.coordinates.w =0
        self.coordinates.h =0
        

    def bottom(self ,msg):
       self.Z_m = msg.ranges[0]
    def marker_id(self ,msg):
       self.marker_id = msg.marker_id 
       print("p",self.marker_id)  

    def image_callback(self,data):
        try:
           
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv2.imshow('h',self.img)
            #img_width=self.img.shape[0]
            
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
            for (self.coordinates.x, self.coordinates.y, self.coordinates.w, self.coordinates.h) in logo:
                cv2.rectangle(self.img, (self.coordinates.x, self.coordinates.y), (self.coordinates.x + self.coordinates.w, self.coordinates.y + self.coordinates.h), (255, 255, 0), 2)
                #cv2.putText(image,''
            
            self.pub.publish(self.coordinates)
            '''focal_length = (img_width/2)/np.tan(1.3962634/2)
            centre_x_pixel = x+ (w/2)
            centre_y_pixel = y+ (h/2)
            #print(logo)
            
            self.marker.err_x_m = centre_x_pixel*self.Z_m/focal_length
            self.marker.err_y_m = centre_y_pixel*self.Z_m/focal_length
            cv2.circle(self.img ,(centre_x_pixel,centre_y_pixel),5,(0,0,255),-1)
            cv2.imshow("t",cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
            cv2.waitKey(25) 
            if (self.marker.err_x_m>0 and self.marker.err_y_m>0 and self.marker.err_x_m!=float("inf") and self.marker.err_y_m!=float("inf")):
                 self.pub.publish(self.marker)
                
            
            print(self.marker)'''
            cv2.imshow("t",cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
            cv2.waitKey(25)
            self.pub.publish(self.coordinates)

        except CvBridgeError as e:
	    print(e)
	    return
 

if __name__ == '__main__' : 
        d=detect_marker()
        rospy.spin()
