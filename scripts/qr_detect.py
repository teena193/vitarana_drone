#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from vitarana_drone.msg import * 
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera 
		self.img = np.empty([]) # This will store the image frame from camera
		self.bridge = CvBridge()
		
		#variable to store the decoded image 		
		self.x = []
		self.l = []

		#publishing the values of box loaction from qr code
		self.location_pub = rospy.Publisher("/edrone/location" , NavSatFix , queue_size = 1)

		#variables to store the latitude, longitude, altitude
		self.location = NavSatFix()
		self.location.latitude = 0.0 
		self.location.longitude = 0.0 
		self.location.altitude = 0.0 


	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image						
			
			try:           
				#decoding the image
				self.x = decode(self.img)
                                print("qr code detected")
				#extracting the value
				self.l = list(self.x[0].data.split(","))
				
				#assigning the value to variables
				self.location.latitude = float(self.l[0])
				self.location.longitude = float(self.l[1])
				self.location.altitude = float(self.l[2])

			except:
				print("no qr code found.....")	
					
		except CvBridgeError as e:
			print(e)
			return
	
		
		#publishing the box location 
		self.location_pub.publish(self.location)
		


if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()


