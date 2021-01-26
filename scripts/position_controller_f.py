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
import pandas as pd

rospy.init_node('position_controller',anonymous=True)


class Position():
   def __init__(self):

       # We first store the required latitude, longitude and altitude in a variable named
       # [latitude required, longitude required, altitude required]
       self.pos_ref = [0.0, 0.0, 0.0]

       # This is the variable where we store the data we get from GPS
       # [latitude current, longitude current, altitude current]
       self.pos_current = [0.0, 0.0, 0.0]

       df = pd.read_csv('manifest.csv',header=None)
       self.data = df.values
      
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

       self.x_e = -1
       self.y_e = -1
       self.fin_loc = [0 ,0 , 0]
       

       # these are the variable we store the values of rcRoll, rcPitch, rcYaw and rcThrottle to publish to the attitude_controller node
       #[rcRoll, rcPitch, rcYaw, rcThrottle]
       self.setpoint_cmd = edrone_cmd()
       self.setpoint_cmd.rcRoll     = 1500.0
       self.setpoint_cmd.rcPitch    = 1500.0
       self.setpoint_cmd.rcYaw      = 1500.0
       self.setpoint_cmd.rcThrottle = 1500.0

       # settings of Kp, Ki and Kd for [latitude, longitude, altitude]
       self.K_p = [ 800*50000,  800*50000, 4000*0.1]
       self.K_i = [   9*0.1,      9*0.1, 950*0.001]
       self.K_d = [ 3319*500000 ,3319*500000, 5000*2]

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
       self.doh = [0 , 0, 0, 0]
      
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
       if msg.ranges[0] >= 0.5 :
		self.front = msg.ranges[0]

       if msg.ranges[1] >= 0.5 :
		self.right = msg.ranges[1]

       if msg.ranges[2] >= 0.5 :
		self.back  = msg.ranges[2]

       if msg.ranges[3] >= 0.5 :
		self.left  = msg.ranges[3]

       
       
   
   def bottom(self ,msg):
       self.bottom_range = msg.ranges[0]
       
       
   def marker_detection(self ,msg):
       self.x = msg.x
       self.y = msg.y
       self.w = msg.w
       self.h = msg.h
       self.l = msg.l
         
       #print(self.x,self.y,self.w,self.h)    

   def Velocity(self,msg):
       self.vel_x = msg.vector.x
       self.vel_y = msg.vector.y
       self.vel_z = msg.vector.z 



   # The PID function , it takes in the required latitude, longitude and altitude
   def PID(self,lat ,lon, alt):
       
       self.direction_of_heading()
       print("doh",self.doh)
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
       self.length_btw_waypoint_latitude =  self.x_to_lat(10)   #length between each waypoints in latitude 
       self.length_btw_waypoint_longitude = self.y_to_long(10)  #length between each waypoints in longitude 
      
       # checking what it has to travel more latitude or longtitude and then creating waypoints depending on that
       if abs(initial_location[0] - final_location[0]) > abs(initial_location[1] - final_location[1]):
       		self.way_points_no = abs(int((initial_location[0] - final_location[0])/ (self.length_btw_waypoint_latitude))) 
                print("waypoints lat")
                print(abs(initial_location[0] - final_location[0]))

       else :
		self.way_points_no = abs(int((initial_location[1] - final_location[1])/ (self.length_btw_waypoint_longitude)))
                print("waypoints long")
                print(abs(initial_location[1] - final_location[1]))

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
       #print('waypoint',self.way_points_longitude)

   # Function for obstacle avoid, it gets data from the range_finder_top and currently its looking only for the direction of travel in this map (ie - left for drone)
   def Obstacle_avoid(self,fin_loc):
       '''ref = 5
       cur_loc = [self.pos_current[0],self.pos_current[1],self.pos_current[2]]
       if self.front <=5:
           obs_location =[0,self.front]
       if self.back <=5:
           obs_location =[0,-self.back]
       if self.left <=5:
           obs_location =[-self.left,0]
       if self.right <=5:
           obs_location =[self.right,0]'''
       cur_loc = [self.pos_current[0],self.pos_current[1],self.pos_current[2]]
       diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
       if (self.doh[0]==1) and (self.right < 10):
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
	   '''while not (self.linear_acc_x <= 0.002 and self.linear_acc_y <= 0.002 and self.linear_acc_z <= 0.002 and self.vel_x <=0.05 and self.vel_y<=0.05 and self.vel_z <= 0.05):
	       self.Stop()
	       print("inside stop")'''
           # while the condtion is True our drone will avoid the obstacle by using a command (here it is backward (according to the drone's orientation))
           while (self.right < 12):
	       
               if diff[1] >0:
                   self.PID(cur_loc[0],self.pos_current[1] + self.y_to_long(10),self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside if")
               else:
                   self.PID(cur_loc[0], self.pos_current[1] - self.y_to_long(10),self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside else")
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0]
	       self.pos_after_obs[1] = self.pos_current[1] - self.y_to_long(10) 
               self.pos_after_obs[2] = self.pos_current[2]
	
       elif (self.doh[3]==1) and (self.front < 10):
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
	   '''while not (self.linear_acc_x <= 0.002 and self.linear_acc_y <= 0.002 and self.linear_acc_z <= 0.002 and self.vel_x <=0.05 and self.vel_y<=0.05 and self.vel_z <= 0.05):
	       self.Stop()
	       print("inside stop")'''
           # while the condtion is True our drone will avoid the obstacle by using a command (here it is backward (according to the drone's orientation))
           while (self.front < 12):
	       
               if diff[0] > 0:
                   self.PID(cur_loc[0]- self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside if front")
               else:
                   self.PID(cur_loc[0] + self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside else front")
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0] 
	       self.pos_after_obs[1] = self.pos_current[1]  
               self.pos_after_obs[2] = self.pos_current[2]


       elif (self.doh[1]==1) and (self.left < 10):
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
	   '''while not (self.linear_acc_x <= 0.002 and self.linear_acc_y <= 0.002 and self.linear_acc_z <= 0.002 and self.vel_x <=0.05 and self.vel_y<=0.05 and self.vel_z <= 0.05):
	       self.Stop()
	       print("inside stop")'''
           # while the condtion is True our drone will avoid the obstacle by using a command (here it is backward (according to the drone's orientation))
           while (self.left < 12):
	       
               if diff[1] > 0:
                   self.PID(cur_loc[0],self.pos_current[1] - self.y_to_long(10),self.stop_loc[2] ) #error 
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside if left")
               else:
                   self.PID(cur_loc[0],self.pos_current[1] + self.y_to_long(10),self.stop_loc [2]) #error
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside else left")
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0] 
	       self.pos_after_obs[1] = self.pos_current[1] - self.y_to_long(10) 
               self.pos_after_obs[2] = self.pos_current[2]

       elif (self.doh[2]==1) and (self.back < 10):
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
	   '''while not (self.linear_acc_x <= 0.002 and self.linear_acc_y <= 0.002 and self.linear_acc_z <= 0.002 and self.vel_x <=0.05 and self.vel_y<=0.05 and self.vel_z <= 0.05):
	       self.Stop()
	       print("inside stop")'''
           # while the condtion is True our drone will avoid the obstacle by using a command (here it is backward (according to the drone's orientation))
           while (self.back < 12):
	       
               if diff[0] > 0:
                   self.PID(cur_loc[0]+ self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside if front")
               else:
                   self.PID(cur_loc[0] - self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   print("inside else front")
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0] 
	       self.pos_after_obs[1] = self.pos_current[1]  
               self.pos_after_obs[2] = self.pos_current[2]



   
    
   def x_to_lat(self, input_x):
       return (input_x / 110692.0702932625)  

   def y_to_long(self, input_y):
       return  (input_y / -105292.0089353767) 
    # if you use this for control, you may have to change the relevant pitch   direction because of the sign
   def lat_to_x(self, input_latitude):
       return 110692.0702932625 * (input_latitude - 19)

   def long_to_y(self, input_longitude):
       return -105292.0089353767 * (input_longitude - 72)
    
    
   #A stop function that stops the drone where ever it is.
   def Stop(self):
       self.PID(self.stop_loc[0],self.stop_loc[1],self.stop_loc[2])
       r.sleep()
   #A forward function so that the drone travels forward (according to drone's orientation)    
   def Forward(self):
       self.PID(self.pos_current[0], self.pos_current[1] - self.length_btw_waypoint_longitude , self.pos_current[2])
       r.sleep()
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
  
   def find_marker(self , building_location):
       # image width
       img_width = 400
       # Horizontal field of view 
       hfov = 1.3962634
       # Calculating the distance the drone will have to travel to get to the marker
       focal_length = (img_width/2)/np.tan(hfov/2)
       centre_x_pixel = self.x+ (self.w/2)
       centre_y_pixel = self.y+ (self.h/2)
       err_x_m = ((200-centre_x_pixel)*(self.pos_current[2]-building_location))/focal_length
       err_y_m =  ((200-centre_y_pixel)*(self.pos_current[2]-building_location))/focal_length
       b = p.y_to_long(err_y_m) # the distace to move in longitude
       a = p.x_to_lat(err_x_m)  # the distace to move in latitude 
       return b,a

   def travel(self , current , destination):

       #destination = [x1 , y1 , z1]
       #initial = [x0 , y0 , z0]
       #pos_current = [x , y , z]

       if (destination[0] >= current[0] and destination[1] >= current[1] and destination[2] >= current[2]):
           condition = (self.pos_current[0]  >= destination[0] and self.pos_current[1] >= destination[1] and self.pos_current[2] >= current[2])
           return condition

       elif (destination[0] >= current[0] and destination[1] >= current[1] and destination[2] <= current[2]):
           condition = (self.pos_current[0]  >= destination[0] and self.pos_current[1] >= destination[1] and self.pos_current[2] <= current[2])
           return condition    

       elif (destination[0] <= current[0] and destination[1] >= current[1] and destination[2] >= current[2]):
           condition = (self.pos_current[0]  <= destination[0] and self.pos_current[1] >= destination[1] and self.pos_current[2] >= current[2])
           return condition

       elif (destination[0] <= current[0] and destination[1] >= current[1] and destination[2] <= current[2]):
           condition = (self.pos_current[0]  <= destination[0] and self.pos_current[1] >= destination[1] and self.pos_current[2] <= current[2])
           return condition

       elif (destination[0] <= current[0] and destination[1] <= current[1] and destination[2] >= current[2]):
           condition = (self.pos_current[0]  <= destination[0] and self.pos_current[1] <= destination[1] and self.pos_current[2] >= current[2])
           return condition

       elif (destination[0] <= current[0] and destination[1] <= current[1] and destination[2] <= current[2]):
           condition = (self.pos_current[0]  <= destination[0] and self.pos_current[1] <= destination[1] and self.pos_current[2] <= current[2])
           return condition

       elif (destination[0] >= current[0] and destination[1] <= current[1] and destination[2] >= current[2]):
           condition = (self.pos_current[0]  >= destination[0] and self.pos_current[1] <= destination[1] and self.pos_current[2] >= current[2])
           return condition

       else : #(destination[0] >= current[0] and destination[1] <= current[1] and destination[2] <= current[2]):
           condition = (self.pos_current[0]  >= destination[0] and self.pos_current[1] <= destination[1] and self.pos_current[2] <= current[2])
           return condition


   def direction_of_heading(self):
       self.doh = [0 ,0 ,0 ,0] # [+lat,-lat,+long,-long]
       if abs(self.vel_x) >= 0.2:
           if self.vel_x <0:
               self.doh[1] = 1  
           else:
               self.doh[0] = 1
       if abs(self.vel_y) >= 0.2:
           if self.vel_y <0:
               self.doh[2] = 1  
           else:
               self.doh[3] = 1
       
       
      
     
       
          
  




if __name__ == '__main__':

    p = Position()
    r = rospy.Rate(30)
    A_1 = [18.9999864489,71.9999430161,8.44099749139]
    B_2 = [18.9999864489 + p.x_to_lat(1.5),71.9999430161 - p.y_to_long(1.5),8.44099749139]#[19.0000000000062, 71.99995726219537, 8.44099749139]
    C_1 = [18.9999864489+p.x_to_lat(3),71.9999430161,8.44099749139]    
    
    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    final_loc = [p.pos_current[0],p.pos_current[1],p.pos_current[2]+1]
    
    while not (p.pos_current[2] >= initial_location[2]+1):
	p.PID(initial_location[0] , initial_location[1] , initial_location[2] + 1)
	r.sleep()

    '''initial_location_at_start = p.initial
    initial_location = [p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    print('starting=',initial_location,'gps=',p.pos_current)
    condition = p.travel(initial_location,A_1)'''
    while not (p.pos_current[0] <= A_1[0] and p.pos_current[1] <=A_1[1] and p.pos_current[2]>= A_1[2] + 1):
        p.PID(A_1[0],A_1[1],A_1[2]+1)
        
        r.sleep()

    while not p.gripper_act == "True":
        p.PID(A_1[0],A_1[1],A_1[2])
        r.sleep()  

    # activates the gripper 		
    x = Gripper()
    x.activate_gripper = True
    activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper )
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is attached to the drone')
    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    while not (p.pos_current[2] >=p.data[0][3]+4):
	p.PID(initial_location[0],initial_location[1],p.data[0][3]+4)
	r.sleep()

    
    initial_loc =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    
    final_loc=[p.data[0][1],p.data[0][2],p.data[0][3]+4]
    k = 1  
    p.Path_Planning(initial_loc,final_loc)   
    while  (k < (p.way_points_no + 2)):
        print('in here while path planning')
        p.PID(p.way_points_latitude[k], p.way_points_longitude[k], p.data[0][3]+4)
        p.Obstacle_avoid(final_loc)
        r.sleep()
        if (p.r == True):
	    p.Path_Planning(p.pos_after_obs, final_loc)
	    k = 1
            p.r = False
        print('k',k)
        #condition so that the drone starts to move to the next waypoint when the last one is achieved
	
        initial_loc = [p.way_points_latitude[k-1], p.way_points_longitude[k-1], p.data[0][3]+4]  
	final_location =  [p.way_points_latitude[k], p.way_points_longitude[k], p.data[0][3]+4] 
        condition = p.travel(initial_loc , final_location)

        if (condition) :
            k+=1  
	    print("p.way_points_no + 2",p.way_points_no + 2)


    i = 1
   
    while p.l == 0:
	
	p.PID(p.data[0][1],p.data[0][2],p.data[0][3] + 1 + 2.5*i)
	r.sleep()
	if p.pos_current[2] >= p.data[0][3] + 1 + 2.5*i :
		i =i+1


    initial_loc = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    b,a = p.find_marker(p.data[0][3])
    final_location = [initial_loc [0]-a , initial_loc [1]+b , initial_loc [2]]
    condition = p.travel(initial_loc  , final_location)

    while not (condition):
	print("hello")
	condition = p.travel(initial_loc , final_location)
	p.PID(initial_loc [0] - a,initial_loc [1] + b,initial_loc[2])
        r.sleep()

    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    while not (p.pos_current[2] <=p.data[0][3]+0.26 +p.bottom_range):
	p.PID(initial_location[0],initial_location[1],p.data[0][3]+0.26 +p.bottom_range)
	r.sleep()

    x.activate_gripper = False
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is droped gently (becuase its not a bomb :D)')


    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    while not (p.pos_current[2] >=p.data[0][3]+4):
	p.PID(initial_location[0],initial_location[1],p.data[0][3]+4)
	r.sleep()
   
    initial_loc =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    
    final_loc=[C_1[0],C_1[1],p.data[0][3]+4]
    k = 1  
    p.Path_Planning(initial_loc,final_loc)   
    while  (k < (p.way_points_no + 2)):
        print('in here while path planning')
        p.PID(p.way_points_latitude[k], p.way_points_longitude[k], p.data[0][3]+4)
        p.Obstacle_avoid(final_loc)
        r.sleep()
        if (p.r == True):
	    p.Path_Planning(p.pos_after_obs, final_loc)
	    k = 1
            p.r = False
        print('k',k)
        #condition so that the drone starts to move to the next waypoint when the last one is achieved
	
        initial_loc = [p.way_points_latitude[k-1], p.way_points_longitude[k-1], p.data[0][3]+4]  
	final_location =  [p.way_points_latitude[k], p.way_points_longitude[k], p.data[0][3]+4] 
        condition = p.travel(initial_loc , final_location)

        if (condition) :
            k+=1  
	    print("p.way_points_no + 2",p.way_points_no + 2)
   
    while not p.gripper_act == "True":
        p.PID(C_1[0],C_1[1],C_1[2])
        r.sleep()
   
    # activates the gripper 		
    x = Gripper()
    x.activate_gripper = True
    activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper )
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is attached to the drone')
    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    while not (p.pos_current[2] >=p.data[1][3]+4):
	p.PID(initial_location[0],initial_location[1],p.data[1][3]+4)
	r.sleep()

    initial_loc =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    
    final_loc=[p.data[1][1],p.data[1][2],p.data[1][3]+4]
    k = 1  
    p.Path_Planning(initial_loc,final_loc)   
    while  (k < (p.way_points_no + 2)):
        print('in here while path planning')
        p.PID(p.way_points_latitude[k], p.way_points_longitude[k], p.data[1][3]+4)
        p.Obstacle_avoid(final_loc)
        r.sleep()

        if (p.r == True):
	    p.Path_Planning(p.pos_after_obs, final_loc)
	    k = 1
            p.r = False
        print('k',k)
        #condition so that the drone starts to move to the next waypoint when the last one is achieved
	
        initial_loc = [p.way_points_latitude[k-1], p.way_points_longitude[k-1], p.data[1][3]+4]  
	final_location =  [p.way_points_latitude[k], p.way_points_longitude[k], p.data[1][3]+4] 
        condition = p.travel(initial_loc , final_location)

        if (condition) :
            k+=1  
	    print("p.way_points_no + 2",p.way_points_no + 2)



    i = 1
   
    while p.l == 0:
	
	p.PID(p.data[0][1],p.data[0][2],p.data[0][3]  + 1*i)
	r.sleep()
	if p.pos_current[2] >= p.data[0][3] + 1*i :
		i =i+1

    initial_loc = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    b,a = p.find_marker(p.data[1][3])
    final_location = [initial_loc [0]-a , initial_loc [1]+b , initial_loc [2]]
    condition = p.travel(initial_loc  , final_location)

    while not (condition):
	print("hello")
	condition = p.travel(initial_loc , final_location)
	p.PID(initial_loc [0] - a,initial_loc [1] + b,initial_loc[2])
        r.sleep()

    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    while not (p.pos_current[2] <=p.data[1][3]+0.26):
	p.PID(initial_location[0],initial_location[1],p.data[1][3])
	r.sleep()

    x.activate_gripper = False
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is droped gently (becuase its not a bomb :D)')
  
    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    while not (p.pos_current[2] >=p.data[1][3]+4):
	p.PID(initial_location[0],initial_location[1],p.data[1][3]+4)
	r.sleep()
  
    initial_loc =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    
    final_loc=[B_2[0],B_2[1],p.data[1][3]+4]
    k = 1  
    p.Path_Planning(initial_loc,final_loc)   
    while  (k < (p.way_points_no + 2)):
        print('in here while path planning')
        p.PID(p.way_points_latitude[k], p.way_points_longitude[k], p.data[1][3]+4)
        p.Obstacle_avoid(final_loc)
        r.sleep()
        if (p.r == True):
	    p.Path_Planning(p.pos_after_obs, final_loc)
	    k = 1
            p.r = False
        print('k',k)
        #condition so that the drone starts to move to the next waypoint when the last one is achieved
	
        initial_loc = [p.way_points_latitude[k-1], p.way_points_longitude[k-1], p.data[1][3]+4]  
	final_location =  [p.way_points_latitude[k], p.way_points_longitude[k], p.data[1][3]+4] 
        condition = p.travel(initial_loc , final_location)

        if (condition) :
            k+=1  
	    print("p.way_points_no + 2",p.way_points_no + 2)
   
   
    while not p.gripper_act == "True":
        p.PID(B_2[0],B_2[1],B_2[2])
        r.sleep()  
    
    x = Gripper()
    x.activate_gripper = True
    activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper )
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is attached to the drone')
    p.fin_loc = [p.data[2][1],p.data[2][2],p.data[2][3]+4]
    initial_location =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    while not (p.pos_current[2] >=p.data[2][3]+4):
	p.PID(initial_location[0],initial_location[1],p.data[2][3]+4)
	r.sleep()
    k=1
   
    while (p.pos_current[2] <= p.fin_loc[2]):
	p.PID(B_2[0] , B_2[1] , p.fin_loc[2])
	r.sleep()
	
    p.Path_Planning(B_2, p.fin_loc)


    while  (k < (p.way_points_no + 2)):
        p.PID(p.way_points_latitude[k], p.way_points_longitude[k],p.fin_loc[2])
        p.Obstacle_avoid(p.fin_loc)
        r.sleep()
        if (p.r == True):
	    p.Path_Planning(p.pos_after_obs, p.fin_loc)
	    k = 1
            p.r = False
        print('k',k)
        #condition so that the drone starts to move to the next waypoint when the last one is achieved
	
    
        initial_loc = [p.way_points_latitude[k-1], p.way_points_longitude[k-1], p.fin_loc[2]]  
	final_location =  [p.way_points_latitude[k], p.way_points_longitude[k], p.fin_loc[2]] 
        condition = p.travel(initial_loc , final_location)

        if (condition) :
            k+=1  
	    print("p.way_points_no + 2",p.way_points_no + 2)

    i = 1
   
    while p.l == 0:
	p.PID(p.data[2][1],p.data[2][2],p.data[2][3] + 1 + 2*i)
	r.sleep()
	if p.pos_current[2] >= p.data[2][3] + 1 + 2*i :
		i =i+1

    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    b,a = p.find_marker(p.data[2][3])
    final_location = [current_location[0]-a , current_location[1]+b , current_location[2]]
    condition = p.travel(current_location , final_location)

    while not (condition):
	print("hello")
	condition = p.travel(current_location , final_location)
	p.PID(current_location[0] - a,current_location[1] + b,current_location[2])
        r.sleep()

    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
          
    while not (p.pos_current[2]<=p.data[2][3]+0.26):
        p.PID(current_location[0],current_location[1],p.data[2][3])
        print("last")
        print(p.pos_current[2])
        r.sleep()
    x.activate_gripper = False
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is droped gently (becuase its not a bomb :D)')
      
    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 

    while (p.pos_current[2] <= p.data[2][3]+2):
	p.PID(current_location[0], current_location[1], p.data[2][3]+2)
	r.sleep()
    

    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    
    k = 1  
    p.Path_Planning(current_location , initial_location_at_start )   
    while  (k < (p.way_points_no + 2)):
        print('in here while path planning')
        p.PID(p.way_points_latitude[k], p.way_points_longitude[k], p.data[2][3]+2)
        p.Obstacle_avoid(initial_location_at_start)
        r.sleep()
        if (p.r == True):
	    p.Path_Planning(p.pos_after_obs, initial_location_at_start)
	    k = 1
            p.r = False
        print('k',k)
        #condition so that the drone starts to move to the next waypoint when the last one is achieved
	
    
        initial_loc = [p.way_points_latitude[k-1], p.way_points_longitude[k-1], p.data[2][3]+2]  
	final_location =  [p.way_points_latitude[k], p.way_points_longitude[k], p.data[2][3]+2] 
        condition = p.travel(initial_loc , final_location)

        if (condition) :
            k+=1  
	    print("p.way_points_no + 2",p.way_points_no + 2)



    while (p.pos_current[2] >= initial_location_at_start[2]):
	p.PID(initial_location_at_start[0],initial_location_at_start[1],initial_location_at_start[2])
	r.sleep()


    


    
    
        
    
       
  
	

     
   
   




 

		

       
