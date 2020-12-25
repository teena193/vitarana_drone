#!/usr/bin/env python

# importing required libraires 

import math 
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import String
from vitarana_drone.srv import Gripper
import numpy as np

rospy.init_node('position_controller',anonymous=True)


class Position():
   def __init__(self):

       # We first store the required latitude, longitude and altitude in a variable named
       # [latitude required, longitude required, altitude required]
       self.pos_ref = [0.0, 0.0, 0.0]

       # This is the variable where we store the data we get from GPS
       # [latitude current, longitude current, altitude current]
       self.pos_current = [0.0, 0.0, 0.0]
	
       self.final_loc = [0.0 ,0.0 ,0.0 ]
       self.gripper_act = ""
       self.k = -1
       self.linear_acc_x = 0.0 
       self.linear_acc_y = 0.0
       self.linear_acc_z = 0.0
       self.i = 0
       self.initial = [0.0 ,0.0 , 0.0]

      	
       self.way_points_latitude = []
       self.way_points_longitude = []
       self.length_btw_waypoint_latitude = 0
       self.length_btw_waypoint_longitude = 0
       self.way_points_no = 0	
       	
       self.front = 0.0
       self.right = 0.0
       self.back  = 0.0
       self.left  = 0.0

       self.r = False	
       self.pos_after_obs = [0.0 , 0.0 , 0.0]
       self.vel_x = 0
       self.vel_y = 0
       self.vel_z = 0

       self.stop_loc = [0.0 , 0.0 ,0.0]

       # these are the variable we store the values of rcRoll, rcPitch, rcYaw and rcThrottle to publish to the attitude_controller node
       #[rcRoll, rcPitch, rcYaw, rcThrottle]
       self.setpoint_cmd = edrone_cmd()
       self.setpoint_cmd.rcRoll     = 1500.0
       self.setpoint_cmd.rcPitch    = 1500.0
       self.setpoint_cmd.rcYaw      = 1500.0
       self.setpoint_cmd.rcThrottle = 1500.0

       # settings of Kp, Ki and Kd for [latitude, longitude, altitude]
       self.K_p = [ 3188*1000,  3188*1000, 4000*0.1]
       self.K_i = [   3*0.1,      3*0.1, 950*0.001]
       self.K_d = [ 5000*100000 ,5000*100000, 5000*2]
       
       # previous values of error for differential part of PID   
       # [latitude previous error, longitude previous error, altitude previous error] 
       self.prev_error_values = [0.0 ,0.0 ,0.0]
       
       # Integral term for PID 
       # [latitude ,longitude, altitude]
       self.iterm = [0.0, 0.0, 0.0]
       
       self.min_values = [1000, 1000, 1000]
       self.max_values = [2000, 2000, 2000]
     
       self.error = [0.0, 0.0, 0.0]
       
       self.pid_terms = [0.0, 0.0, 0.0]
       self.obs_detect_loc = [0.0, 0.0]
      
       self.height_req_for_qr = 1.2
       self.box_height = 0.31
       self.bottom_range=0.0
       
       self.x = 0.0
       self.y = 0.0
      
       #publishers
       self.cmd_publish = rospy.Publisher('/drone_command',edrone_cmd,queue_size = 1)

       #subcription
       rospy.Subscriber('/edrone/gps',NavSatFix,self.Nav_data)
       rospy.Subscriber("/edrone/location" , NavSatFix , self.final_dest) 
       rospy.Subscriber('/edrone/gripper_check' , String , self.gripper)
       rospy.Subscriber('/edrone/imu/data', Imu, self.distance)
       rospy.Subscriber('/edrone/range_finder_top' , LaserScan , self.Ranges)
       rospy.Subscriber('/edrone/gps_velocity' , Vector3Stamped , self.Velocity)
       rospy.Subscriber('/edrone/range_finder_bottom' , LaserScan , self.bottom)
       rospy.Subscriber('/edrone/coordinate_img',coordinates, self.marker_detection)
       self.pub = rospy.Publisher('/marker_id',MarkerData,queue_size = 1)
   #defining callback_function
       self.marker = MarkerData()
       self.marker.marker_id =int(0)
   
   # This is a callback function to store the data published by GPS into class variables 
   def Nav_data(self,msg):
       self.pos_current[0] = msg.latitude
       self.pos_current[1] = msg.longitude
       self.pos_current[2] = msg.altitude
       if (self.i <= 1 ):
		self.initial[0] = msg.latitude
		self.initial[1] = msg.longitude
		self.initial[2] = msg.altitude	
		self.i = self.i+1
      


   def final_dest(self , msg):
       self.final_loc[0] = msg.latitude
       self.final_loc[1] = msg.longitude
       self.final_loc[2] = msg.altitude
       self.k = self.final_loc[0]
       
   def gripper(self , msg):
       self.gripper_act = msg.data

   def distance(self ,msg):
       self.linear_acc_x = msg.linear_acceleration.x
       self.linear_acc_y = msg.linear_acceleration.y
       self.linear_acc_z = msg.linear_acceleration.z

   def Ranges(self ,msg):
       self.front = msg.ranges[0]
       self.right = msg.ranges[1]
       self.back  = msg.ranges[2]
       self.left  = msg.ranges[3]
   
   def bottom(self ,msg):
       self.bottom_range = msg.ranges[0]
       
       
   def marker_detection(self ,msg):
       self.x = msg.x
       self.y = msg.y
       self.w = msg.w
       self.h = msg.h
         
       #print(self.x,self.y,self.w,self.h)    

   def Velocity(self,msg):
       self.vel_x = msg.vector.x
       self.vel_y = msg.vector.y
       self.vel_z = msg.vector.z 



   # The PID function , it takes in the required latitude, longitude and altitude
   def PID(self,lat ,lon, alt):
       
       # we set the required position
       self.pos_ref = [lat, lon, alt]
       
       # calculating error in position
       self.error[0] = self.pos_ref[0] - self.pos_current[0]
       self.error[1] = self.pos_ref[1] - self.pos_current[1]
       self.error[2] = self.pos_ref[2] - self.pos_current[2]
      
       # calculating Integral terms of Latitude, Longitude and Altitude 
       self.iterm[0] = ( self.iterm[0] + self.error[0] ) * self.K_i[0]
       self.iterm[1] = ( self.iterm[1] + self.error[1] ) * self.K_i[1]
       self.iterm[2] = ( self.iterm[2] + self.error[2] ) * self.K_i[2]
       
       # Calculating PID terms from the error terms and PID constants 
       self.pid_terms[0] = self.error[0] * self.K_p[0] +self.iterm[0] + self.K_d[0] * (self.error[0] - self.prev_error_values[0])
       self.pid_terms[1] = self.error[1] * self.K_p[1] +self.iterm[1] + self.K_d[1] * (self.error[1] - self.prev_error_values[1])
       self.pid_terms[2] = self.error[2] * self.K_p[2] +self.iterm[2] + self.K_d[2] * (self.error[2] - self.prev_error_values[2])
          
       # setting the value of rcRoll, rcPitch, rcYaw and rcThrottle based on our PID terms 
       self.setpoint_cmd.rcRoll     = 1500 + self.pid_terms[0]
       self.setpoint_cmd.rcPitch    = 1500 + self.pid_terms[1]
       self.setpoint_cmd.rcYaw      = 1500 
       self.setpoint_cmd.rcThrottle = 1500 + self.pid_terms[2]

       # Limiting the value of commands to a minimum of 1000
       if self.setpoint_cmd.rcRoll > self.max_values[0]:
           self.setpoint_cmd.rcRoll = self.max_values[0]
       if self.setpoint_cmd.rcPitch > self.max_values[1]:
           self.setpoint_cmd.rcPitch = self.max_values[1]
       if self.setpoint_cmd.rcThrottle > self.max_values[2]:
           self.setpoint_cmd.rcThrottle = self.max_values[2]
 
       # Limiting the value of command to a maximum of 2000
       if self.setpoint_cmd.rcRoll < self.min_values[0]:
           self.setpoint_cmd.rcRoll = self.min_values[0]
       if self.setpoint_cmd.rcPitch < self.min_values[1]:
           self.setpoint_cmd.rcPitch = self.min_values[1]
       if self.setpoint_cmd.rcThrottle < self.min_values[2]:
           self.setpoint_cmd.rcThrottle = self.min_values[2]
       
       # Storing the current value of error as previous error to be used later.
       self.prev_error_values[0] = self.error[0]
       self.prev_error_values[1] = self.error[1]
       self.prev_error_values[2] = self.error[2]      
       
       # publishing the command values that will be subscribed by attitude_controller 
       self.cmd_publish.publish(self.setpoint_cmd)
  
     
   #This is a path planning function, it takes 2 location and gives the shortest path between them.
   def Path_Planning(self, initial_location, final_location): 
     
       self.way_points_latitude = []
       self.way_points_longitude = []
       self.length_btw_waypoint_latitude = 0.0000440429*2    #length between each waypoints in latitude 
       self.length_btw_waypoint_longitude = 0.0000127268*2   #length between each waypoints in longitude 
      
       # checking what it has to travel more latitude or longtitude and then creating waypoints depending on that
       if abs(initial_location[0] - final_location[0]) > abs(initial_location[1] - final_location[1]):
       		self.way_points_no = abs(int((initial_location[0] - final_location[0])/ (self.length_btw_waypoint_latitude))) 

       else :
		self.way_points_no = abs(int((initial_location[1] - final_location[1])/ (self.length_btw_waypoint_longitude)))

       #creating waypoints for latitude 
       for i in range(self.way_points_no + 2):                                                                      
	   if (initial_location[0] >= final_location[0]): 
	       self.way_points_latitude.append(initial_location[0] - (self.length_btw_waypoint_latitude) * i)
	
	   else:
               self.way_points_latitude.append(initial_location[0] + (self.length_btw_waypoint_latitude) * i)
	
       self.way_points_latitude[-1] = final_location[0]  #we store the final waypoint as final location so that the drone always goes to it

       #creating waypoints for longitude 
       for i in range (self.way_points_no + 2): 
           self.length_btw_waypoint_longitude = abs((initial_location[1] - final_location[1]))/(self.way_points_no)                                                             
	   if (initial_location[1] >= final_location[1]):  
	       self.way_points_longitude.append(initial_location[1] - (self.length_btw_waypoint_longitude) * i)
	
	   else :
               self.way_points_longitude.append(initial_location[1] + (self.length_btw_waypoint_longitude) * i)	

       self.way_points_longitude[-1] = final_location[1]  #we store the final waypoint as final location so that the drone always goes to it
   

   # Function for obstacle avoid, it gets data from the range_finder_top and currently its looking only for the direction of travel in this map (ie - left for drone)
   def Obstacle_avoid(self):
       
       #checks if the distance between obstacle and drone is less than 10 then its avoids the obstacle 
       if (self.left < 8):
           #we put this self.r variable true so if it becomes true, we can indicate to our path planner that it needs to create a new path after the obstacle is avoided 
	   self.r = True
 	   self.stop_loc = [self.pos_current[0], self.pos_current[1], self.pos_current[2]]
           # as long as the drone is not stable it will try to stop it by using the function Stop() defined below, and to check if it stable we use data from imu linear acceration and gps velocity.
	   while not (self.linear_acc_x <= 0.0002 and self.linear_acc_y <= 0.0002 and self.linear_acc_z <= 0.0002 and self.vel_x <=0.05 and self.vel_y<=0.05 and self.vel_z <= 0.05):

               self.Stop()
               print('Stopping')
               r.sleep()
           # while the condtion is True our drone will avoid the obstacle by using a command (here it is backward (according to the drone's orientation))
           while (self.left < 10):

               self.Backward()
               r.sleep()
	       print('obstacle avoiding...')

	       self.pos_after_obs[0] = self.pos_current[0]
	       self.pos_after_obs[1] = self.pos_current[1] + self.length_btw_waypoint_longitude  #we add this length to compensate for the fact that our drone has some length protruding from where the range finder sensor
	       self.pos_after_obs[2] = self.pos_current[2]
              
           self.stop_loc = [self.pos_current[0],self.pos_current[1] +self.length_btw_waypoint_longitude,self.pos_current[2]]

           while not (self.linear_acc_x <= 0.0002 and self.linear_acc_y <= 0.0002  and self.vel_x <=0.05 and self.vel_y<=0.05 ):

               self.Stop()
               print('Stopping')
               r.sleep()
   def x_to_lat(self, input_x):
       return (input_x / 110692.0702932625)  

   def y_to_long(self, input_y):
       return  (input_y / -105292.0089353767) 
    # if you use this for control, you may have to change the relevant pitch   direction because of the sign
    
    
   #A stop function that stops the drone where ever it is.
   def Stop(self):
       self.PID(self.stop_loc[0],self.stop_loc[1],self.stop_loc[2])
   #A forward function so that the drone travels forward (according to drone's orientation)    
   def Forward(self):
       self.PID(self.pos_current[0], self.pos_current[1] - self.length_btw_waypoint_longitude , self.pos_current[2])
   #A backward function so that the drone travels backward (according to drone's orientation)
   def Backward(self):
       self.PID(self.pos_current[0] , self.pos_current[1] + self.length_btw_waypoint_longitude , self.stop_loc[2])
   #A Right function so that the drone travels to the right (according to drone's orientation)
   def Right(self):
       self.PID(self.pos_current[0] + self.length_btw_waypoint_latitude, self.pos_current[1] , self.pos_current[2])
   #A Left function so that the drone travels to the left (according to drone's orientation)	
   def Left(self):
       self.PID(self.pos_current[0] - self.length_btw_waypoint_latitude, self.pos_current[1] , self.pos_current[2])
   #An Up function so that the drone travels Up (according to drone's orientation)	
   def Up(self):
       self.PID(self.pos_current[0] , self.pos_current[1] , self.pos_current[2] + 0.2)      
   #A down function so that the drone travels down (according to drone's orientation)
   def Down(self):
       self.PID(self.pos_current[0] , self.pos_current[1] , self.pos_current[2] - 0.2)
  
   def find_marker(self):
       # image width
       img_width = 400
       # Horizontal field of view 
       hfov = 1.3962634
       # Calculating the distance the drone will have to travel to get to the marker
       focal_length = (img_width/2)/np.tan(hfov/2)
       centre_x_pixel = self.x+ (self.w/2)
       centre_y_pixel = self.y+ (self.h/2)
       err_x_m = ((200-centre_x_pixel)*p.bottom_range)/focal_length
       err_y_m =  ((200-centre_y_pixel)*p.bottom_range)/focal_length
       b = p.y_to_long(err_y_m) # the distace to move in longitude
       a = p.x_to_lat(err_x_m)  # the distace to move in latitude 
       return b,a



if __name__ == '__main__':

    p = Position()
    r = rospy.Rate(30)
    
    # Given location of the buildings --- [latitude, longitude, altitude]
    building_1 = [18.9990965928, 72.0000664814, 10.75] 
    building_2 = [18.9990965925, 71.9999050292,  22.2]
    building_3 = [18.9993675932, 72.0000569892,  10.7]
    
    # Edrone going up
    while (p.pos_current[2] <= p.initial[2]+1):
	p.PID(p.initial[0] ,p.initial[1] , p.initial[2]+1)
        print('going up by 1m')
        r.sleep()
    # Storing the marker id(ie = 3) and publishing it    
    p.marker.marker_id =int(3)
    p.pub.publish(p.marker)
    
    # Using range_finder_bottom so as to aviod going down when there is building below that we dont not want to land on.
    while (p.bottom_range<=1.4):
        p.PID( building_3[0], building_3[1],p.initial[2]+1)
        print('building below!!')
        r.sleep()
    # Going to building 3 
    while (p.pos_current[0]<=building_3[0] and p.pos_current[1]<=building_3[1] and p.pos_current[2]>=building_3[2]+1):
        p.PID( building_3[0], building_3[1],building_3[2]+1)
        print('building 3 ...')
        r.sleep()
    # seraching for the marker by going up by 14m 
    while (p.pos_current[2]<=building_3[2]+14):
        # Calculating the distance the drone will have to travel to get to the marker
	b,a = p.find_marker()

        p.PID( building_3[0], building_3[1],building_3[2]+14)
        print('searching...')
        r.sleep()

    # Calculating the distance the drone will have to travel to get to the marker
    b,a = p.find_marker()
  
    # Navigating to the Marker
    while not (p.pos_current[0]<=building_3[0] - a and p.pos_current[1]>=building_3[1] +b):
        p.PID( building_3[0] - a, building_3[1] + b, building_3[2]+14)
        print('Navigating to the Marker')
        r.sleep()
    #going down to hover at 1m above the marker
    '''while  (p.pos_current[2] >= building_3[2] +1.01):
        p.PID( building_3[0] - a, building_3[1] +b, building_3[2]+1)
        print('hover above marker...')
        r.sleep()'''

    # Storing the marker id(ie = 1) and publishing it 
    p.marker.marker_id =int(1)
    p.pub.publish(p.marker)
    
     # Going to building 1 
    while not  (p.pos_current[1]>=building_1[1] and p.pos_current[2]>=building_1[2]):
        p.PID( building_1[0], building_1[1], building_1[2]+1) 
        print('building 1...')
        r.sleep() 
      
    # seraching for the marker by going up by 14m 
    while (p.pos_current[2]<=building_1[2]+14):
	b,a = p.find_marker()
        p.PID( building_1[0], building_1[1],building_1[2]+14)
        print('searching')
        r.sleep()
    # Calculating the distance the drone will have to travel to get to the marker
    b,a = p.find_marker()

    # Navigating to the Marker
    while (p.pos_current[0]>= building_1[0] - a and p.pos_current[1]<=building_1[1] + b) :
        p.PID( building_1[0] -a , building_1[1] +b ,building_1[2]+14)
        print('Navigating to the Marker...')
        r.sleep()
    # going down to hover at 1m above the marker
    '''while (p.pos_current[2] >= building_1[2] + 1.05) :
        p.PID(building_1[0] - a,building_1[1] + b, building_1[2] + 1)
        print('hover above marker...')
        r.sleep()'''

    # Storing the marker id(ie = 2) and publishing it 
    p.marker.marker_id =int(2)
    p.pub.publish(p.marker)
    
    # Going to building 2
    while not (p.pos_current[0] <= building_2[0] and p.pos_current[1] <= building_2[1]):
        p.PID(building_2[0],building_2[1],building_1[2] +14)
        print('building 2...')
        r.sleep()
      
    while (p.pos_current[2] >=building_2[2] +1.05):
        p.PID(building_2[0],building_2[1],building_2[2] +1)
        print('building 2...')
        r.sleep()
       
    # traveling 5m in longitude   
    while (p.pos_current[1] >= building_2[1] + p.y_to_long(5)):
        p.PID(building_2[0],building_2[1]+ p.y_to_long(5),building_2[2] +1)
        print('searching...')
        r.sleep()
    
    # searching for the marker by going up by 14m 
    while p.pos_current[2] <= building_2[2] + 14:
        b,a = p.find_marker()
        p.PID(building_2[0],building_2[1]+ p.y_to_long(5),building_2[2] +14)
        print('searching...')
        r.sleep()

    # Calculating the distance the drone will have to travel to get to the marker   
    b,a = p.find_marker()
    
    # Navigating to the Marker
    while not (p.pos_current[0]>=building_2[0] -a and p.pos_current[1]<=building_2[1] + p.y_to_long(5) +b):
        p.PID(building_2[0]-a,building_2[1]+ p.y_to_long(5) +b,building_2[2]+14)
        print('Navigating to the Marker...')
        r.sleep()  
   # Landing on the marker   
    while not rospy.is_shutdown():
        p.PID(building_2[0]-a,building_2[1]+ p.y_to_long(5) +b,building_2[2])
        print('landing...')
        r.sleep()

   




 

		

       
