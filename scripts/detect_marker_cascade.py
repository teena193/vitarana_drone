#!/usr/bin/env python

#importing all the required libraries
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
        self.img = np.empty([])  # Variable to store image data
        
        # Variable to store the value of range_finder_bottom distance
        self.Z_m = 0.0
        #image width
        self.img_width = 400
        #horizontal field of view
        self.hfov = 1.3962634
        # Variables to store the pixel cordinates of landing marker 
        self.coordinates = coordinates() 

        self.coordinates.x =0
        self.coordinates.y =0
        self.coordinates.w =0
        self.coordinates.h =0
	self.coordinates.l =0
        
        # Variables to store Marker Data
        self.Marker_d = MarkerData()
        self.Marker_d.marker_id = int(0) 
        self.Marker_d.err_x_m = 0.0
        self.Marker_d.err_y_m = 0.0

        self.bridge = CvBridge() 
        # Usings Cascade classifier with trained data from the xml file
        self.logo_cascade = cv2.CascadeClassifier('data/cascade.xml')
   
        # Subscribing 
        rospy.Subscriber('/marker_id',MarkerData, self.marker_id) # Subscribing to the /marker_id that is being published from the position_controller script
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) # Subscribing to the camera of the edrone
        rospy.Subscriber('/edrone/range_finder_bottom' , LaserScan , self.bottom) # Subscribing to the range_finder_bottom sensor of the edrone

        self.pub = rospy.Publisher('/edrone/coordinate_img', coordinates ,queue_size = 1) # Creating a Publisher to Publish the pixel coordinates of lander marker to be used for calculation.
        self.Marker_data = rospy.Publisher('/edrone/marker_data',MarkerData,queue_size = 1) # Creating a publisher to Publish the marker_id, err_x_m and err_y_m
         
    # Callback function for range_finder_bottom sensor
    def bottom(self ,msg):
       self.Z_m = msg.ranges[0]

    # Callback function for marker_id 
    def marker_id(self ,msg):
       self.marker_id = int(msg.marker_id) 

    # Callback funtion for image sensor   
    def image_callback(self,data):
        try:
            # Storing the image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Converting the image to greyscale
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            # Using the cascade Classifier to detect the lander marker in the image
            logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
            # Drawing a rectangle over the image where lander marker is found
            for (self.coordinates.x, self.coordinates.y, self.coordinates.w, self.coordinates.h) in logo:
		self.coordinates.l = 1
                cv2.rectangle(self.img, (self.coordinates.x, self.coordinates.y), (self.coordinates.x + self.coordinates.w, self.coordinates.y + self.coordinates.h), (255, 255, 0), 2)

            # Publishing the pixel coordinates of the lander marker.
            self.pub.publish(self.coordinates)
	    self.coordinates.l =0
            # Calculating focal length
            focal_length = (self.img_width/2)/np.tan(self.hfov/2)
            # Centre pixels of the lander marker
	    centre_x_pixel = self.coordinates.x+ (self.coordinates.w/2)
	    centre_y_pixel = self.coordinates.y+ (self.coordinates.h/2)
            # Storing Marker id
            self.Marker_d.marker_id = (self.marker_id)

            # Calculating the X and Y distace of the lander marker form the edrone 
	    self.Marker_d.err_x_m = (((self.img_width/2)-centre_x_pixel)*self.Z_m)/focal_length
	    self.Marker_d.err_y_m = (((self.img_width/2)-centre_y_pixel)*self.Z_m)/focal_length

            # Publishing the Marker data 
            self.Marker_data.publish(self.Marker_d)

            # Showing images in the format of a video
            cv2.imshow("camera view",cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
            cv2.waitKey(25)

        except CvBridgeError as e:
	    print(e)
	    return
 

if __name__ == '__main__' : 
        d=detect_marker()
        rospy.spin()
