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

       # getting building location from the csv file using pandas 
       df = pd.read_csv('manifest.csv',header=None)
       self.data = df.values

       self.D_f = []
       self.D_t = []

       for i in range(len(self.data)):
           if self.data[i][0] == 'DELIVERY':
               self.D_f.append(self.data[i][1])
               self.D_t.append(self.data[i][2])

       self.numbers=[]

       for j in range(len(self.D_t)):
           self.numbers.append(self.D_t[j].split(';'))
      

       for j in range(len(self.numbers)):
           for i in range(len(self.numbers[j])):
               self.numbers[j][i]= float(self.numbers[j][i])
       

       self.R_f = []
       self.R_t = []

       for i in range(len(self.data)):
           if self.data[i][0] == 'RETURN ':
               self.R_f.append(self.data[i][1])
               self.R_t.append(self.data[i][2])

       print("R_f",self.R_f)       
       print("R_t",self.R_t)

       self.R_numbers=[]

       for j in range(len(self.R_f)):
           self.R_numbers.append(self.R_f[j].split(';'))
       print('a',self.R_numbers)

       for j in range(len(self.R_numbers)):
           for i in range(len(self.R_numbers[j])):
               self.R_numbers[j][i]= float(self.R_numbers[j][i])
       print('b',self.R_numbers)

       print(self.R_numbers)


       self.D = {}

       self.D['A1'] = [18.9998102845 ,72.000142461 ,16.757981]
       self.D['A2'] = [18.9998102845 ,72.000142461- self.y_to_long(1.5) ,16.757981]
       self.D['A3'] = [18.9998102845  ,72.000142461 - (2*self.y_to_long(1.5)) ,16.757981]
   
       self.D['B1'] = [18.9998102845 + self.x_to_lat(1.5) ,72.000142461,16.757981]
       self.D['B2'] = [18.9998102845 + self.x_to_lat(1.5) ,72.000142461 - self.y_to_long(1.5) ,16.757981]
       self.D['B3'] = [18.9998102845 + self.x_to_lat(1.5) ,72.000142461 - (2*self.y_to_long(1.5)) ,16.757981]
   
       self.D['C1'] = [18.9998102845 + (2*self.x_to_lat(1.5)) ,72.000142461 ,16.757981]
       self.D['C2'] = [18.9998102845 + (2*self.x_to_lat(1.5)) ,72.000142461 - self.y_to_long(1.5) ,16.757981]
       self.D['C3'] = [18.9998102845 + (2*self.x_to_lat(1.5)) ,72.000142461 - (2*self.y_to_long(1.5)) ,16.757981]

       self.D['X1 '] = [ 18.9999367615 ,72.000142461 ,16.757981 ]
       self.D['X2 '] = [ 18.9999367615 ,72.000142461- self.y_to_long(1.5) ,16.757981 ]
       self.D['X3 '] = [ 18.9999367615 ,72.000142461- (2*self.y_to_long(1.5)) ,16.757981 ]
      
       self.D['Y1 '] = [ 18.9999367615+ self.x_to_lat(1.5) ,72.000142461 ,16.757981 ]
       self.D['Y2 '] = [ 18.9999367615+ self.x_to_lat(1.5) ,72.000142461- self.y_to_long(1.5) ,16.757981 ]
       self.D['Y3 '] = [ 18.9999367615+ self.x_to_lat(1.5) ,72.000142461- (2*self.y_to_long(1.5)) ,16.757981 ]

       self.D['Z1 '] = [ 18.9999367615+ (2*self.x_to_lat(1.5)) ,72.000142461 ,16.757981 ]
       self.D['Z2 '] = [ 18.9999367615+ (2*self.x_to_lat(1.5)) ,72.000142461- self.y_to_long(1.5) ,16.757981 ]
       self.D['Z3 '] = [ 18.9999367615+ (2*self.x_to_lat(1.5)) ,72.000142461- (2*self.y_to_long(1.5)) ,16.757981 ]
     
       
       self.distance_m = []

       print(self.numbers)
       for i in range(len(self.numbers)):
           j = self.D_f[i]
           s = math.sqrt(((self.lat_to_x(self.numbers[i][0]) - self.lat_to_x(self.D[j][0]))**2) + ((self.long_to_y(self.numbers[i][1]) - self.long_to_y(self.D[j][1]))**2))
           #lat = (self.lat_to_x(self.numbers[i][0]) - self.lat_to_x(self.D[j][0]))**2
           #lon = (self.long_to_y(self.numbers[i][1]) - self.long_to_y(self.D[j][1]))**2
           #print('s',lat,lon)  
           self.distance_m.append(s)
          
           print(self.numbers[i][0])
           print(self.D[j][0])

       
       self.Z = [x for _,x in sorted(zip(self.distance_m,self.D_f))]
       #print("D_f",self.D_f)

       self.Y = [x for _,x in sorted(zip(self.distance_m,self.numbers))]
       #print("D_t",self.D_t)
       #print("Y",self.Y)
       

      
       #decraling some variables to be used 
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
       self.K_p = [ 3000*1000,  3000*1000, 4000*0.1]
       self.K_i = [   9.9*0.1,      9.9*0.1, 950*0.001]
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

   def Tuning(self, current , destination):
       
       latitude = abs(current[0] - destination[0])
       longitude = abs(current[1] - destination[1])

       #short latitude 
       if latitude <= abs(self.x_to_lat(12)):
           self.K_p[0] = 3250*1000  
           self.K_i[0] = 9*0.1   
           self.K_d[0] = 5000*100000 
	   print("tuned for short latitude")

       #long latitude
       else:
	   print("tuned for long latitude")
           pass

       #short longitude
       if longitude <= abs(self.y_to_long(12)):
           self.K_p[1] = 3250*1000  
           self.K_i[1] = 9*0.1   
           self.K_d[1] = 5000*100000 
	   print("tuned for short longitude")

       #long latitude
       else:
	   print("tuned for long longitude")
           pass

       


   # The PID function , it takes in the required latitude, longitude and altitude
   def PID(self,lat ,lon, alt):
       
       # gives direction of heading of drone 
       self.direction_of_heading()
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

   # Function for obstacle avoid, it gets data from the range_finder_top and currently its looking only for the direction of travel.
   def Obstacle_avoid(self,fin_loc):
       cur_loc = [self.pos_current[0],self.pos_current[1],self.pos_current[2]]
       diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
       if (self.doh[0]==1) and (self.right < 10) :
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
           # while the condtion is True our drone will avoid the obstacle by using a command 
           while (self.right < 12):
	       
               if diff[1] >0:
                   self.PID(cur_loc[0],self.pos_current[1] + self.y_to_long(10),self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		   
               else:
                   self.PID(cur_loc[0], self.pos_current[1] - self.y_to_long(10),self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		  
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0]
	       self.pos_after_obs[1] = self.pos_current[1] - self.y_to_long(10) 
               self.pos_after_obs[2] = self.pos_current[2]
	
       elif (self.doh[3]==1) and (self.front < 10):
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
	
           # while the condtion is True our drone will avoid the obstacle by using a command 
           while (self.front < 12):
	       
               if diff[0] > 0:
                   self.PID(cur_loc[0]- self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		  
               else:
                   self.PID(cur_loc[0] + self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		  
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0] 
	       self.pos_after_obs[1] = self.pos_current[1]  
               self.pos_after_obs[2] = self.pos_current[2]


       elif (self.doh[1]==1) and (self.left < 10) :
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
	   
           # while the condtion is True our drone will avoid the obstacle by using a command 
           while (self.left < 12):
	       
               if diff[1] > 0:
                   self.PID(cur_loc[0],self.pos_current[1] - self.y_to_long(10),self.stop_loc[2] ) #error 
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		  
               else:
                   self.PID(cur_loc[0],self.pos_current[1] + self.y_to_long(10),self.stop_loc [2]) #error
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		 
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0] 
	       self.pos_after_obs[1] = self.pos_current[1] - self.y_to_long(10) 
               self.pos_after_obs[2] = self.pos_current[2]

       elif (self.doh[2]==1) and (self.back < 10) :
           self.r = True
	   self.stop_loc = [ self.pos_current[0] , self.pos_current[1] , self.pos_current[2] ]
	   
           # while the condtion is True our drone will avoid the obstacle by using a command
           while (self.back < 12):
	       
               if diff[0] > 0:
                   self.PID(cur_loc[0]+ self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		  
               else:
                   self.PID(cur_loc[0] - self.x_to_lat(10) , cur_loc[1] , self.stop_loc[2])
                   diff = [self.lat_to_x(self.pos_current[0]) - self.lat_to_x(fin_loc[0]),self.long_to_y(self.pos_current[1]) - self.long_to_y(fin_loc[1]),self.pos_current[2] - fin_loc[2]]
                   r.sleep()
		  
	       print('obstacle avoiding...',self.pos_after_obs)
               self.pos_after_obs[0] = self.pos_current[0] 
	       self.pos_after_obs[1] = self.pos_current[1]  
               self.pos_after_obs[2] = self.pos_current[2]



   
   # converting meters to latitude  
   def x_to_lat(self, input_x):
       return (input_x / 110692.0702932625)  
   # converting meters to longitude 
   def y_to_long(self, input_y):
       return  (input_y / -105292.0089353767) 
   # converting latitude to meters 
   def lat_to_x(self, input_latitude):
       return 110692.0702932625 * (input_latitude - 19)
   # converting longitude to meters
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
  
   def find_marker(self,building_location):
       # image width
       img_width = 400
       # Horizontal field of view 
       hfov = 1.3962634
       # Calculating the distance the drone will have to travel to get to the marker
       focal_length = (img_width/2)/np.tan(hfov/2)
       centre_x_pixel = self.x+ (self.w/2)
       centre_y_pixel = self.y+ (self.h/2)
       err_x_m = ((200-centre_x_pixel)*(self.pos_current[2] - building_location))/focal_length
       err_y_m =  ((200-centre_y_pixel)*(self.pos_current[2] - building_location))/focal_length
       b = p.y_to_long(err_y_m) # the distace to move in longitude
       a = p.x_to_lat(err_x_m)  # the distace to move in latitude 
       return b,a

   # this function checks the condition for position of the drone and if it has reached the specified location 
   def travel(self , current , destination):
       

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

       else : 
           condition = (self.pos_current[0]  >= destination[0] and self.pos_current[1] <= destination[1] and self.pos_current[2] <= current[2])
           return condition

   # this function provides the direction of heading of the drone 1 is heading in that direction and 0 is not heading in that direction 
   def direction_of_heading(self):
       self.doh = [0 ,0 ,0 ,0 ] # [+lat,-lat,+long,-long]
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
   
   
   

   def Return(self,current):
       self.R_distance = []
       for j in range(len(self.R_numbers)):
           t = math.sqrt(((self.lat_to_x(self.R_numbers[j][0]) - self.lat_to_x(current[0]))**2) + ((self.long_to_y(self.R_numbers[j][1]) - self.long_to_y(current[1]))**2))
	   self.R_distance.append(t)
       #print("R_distance",self.R_distance)

       #R_f = distance   ,   R_t = ['X1','X2'......]
       self.U = [x for _,x in sorted(zip(self.R_distance,self.R_numbers))]
       #print("R_f",self.R_f)
       self.W = [x for _,x in sorted(zip(self.R_distance,self.R_t))]
       #print("R_t",self.R_t)
       self.R_numbers.remove(self.U[0])
       self.R_t.remove(self.W[0])
       #LOCATION =[p.D[p.W[0]][0],p.D[p.W[0]][1],p.D[p.W[0]][2]+4]
       print('u,w',self.U,self.W)


if __name__ == '__main__':

    p = Position()
    r = rospy.Rate(100)
    current = p.initial

    height = 25

    for i in range(len(p.Z)):
        if i > 0:
        	current = [p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    	final=[p.D[p.Z[i]][0],p.D[p.Z[i]][1],height]
    	condition = p.travel(current,final)
    	print(current)
    	print(final)
        print(p.Z[i],p.Y[i])

        
    	while not p.pos_current[2] >= height:
        	p.PID(current[0],current[1],height)
        	r.sleep()
        
        p.Tuning(current,final)
    	while not condition:
		p.PID(final[0],final[1],final[2])
        	condition = p.travel(current,final)
		print("above box")
		r.sleep()

    	final=[p.D[p.Z[i]][0],p.D[p.Z[i]][1],p.D[p.Z[i]][2]] 
    	#drone goes down until it is ready to pick up the box
    	while not p.gripper_act == "True":
     	        p.PID(final[0],final[1],final[2])
        	print('drone goes down until it is ready to pick up the box')
        	r.sleep()  

    	# activates the gripper  		
    	x = Gripper()
    	x.activate_gripper = True
    	activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper )
    	activate.wait_for_service()
    	activate.call(x.activate_gripper)
    	print('box is attached to the drone')

    
 
    	current = [p.pos_current[0] , p.pos_current[1] , height]
    	final=[p.Y[i][0],p.Y[i][1],height]
    	condition = p.travel(current,final)
    
    	while not p.pos_current[2] >= height:
        	p.PID(current[0],current[1],height)
        	r.sleep()
 
        p.Tuning(current,final)
    	while not condition:
		p.PID(final[0],final[1],final[2])
        	condition = p.travel(current,final)
		print("Building "+p.Z[i])
		r.sleep()
                #if p.l == 1 and (abs(p.pos_current[0] - final[0]) <= p.x_to_lat(8) or abs(p.pos_current[1] - final[1]) <= p.y_to_long(8)):
                    #break
                    #print('nahiiiiiiiiii')
    	current = [p.pos_current[0] , p.pos_current[1] ,p.pos_current[2]] 
    	#searching algorithm
    	o = 1
    	while p.l == 0:
	
		p.PID(final[0] , final[1] , current[2] +5*o)
		r.sleep()
        	print('going up until marker is found..')
		if p.pos_current[2] >= current[2]  + 5*o :
			o =o+1

    	# condition for going to the marker 
    	current = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    	b,a = p.find_marker(p.Y[i][2])
    	final = [current[0]-a , current[1]+b , current[2]]
    	condition = p.travel(current,final)
 
    	# going to the marker 
        p.Tuning(current,final)
    	while not (condition):
	
		condition = p.travel(current,final)
		p.PID(final[0] ,final[1] ,final[2])
        	print('going to the marker...')
        	r.sleep()

    	#going down to the marker
    	current =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    	final=[p.Y[i][0],p.Y[i][1],p.Y[i][2]]

    	while not (p.pos_current[2] <= final[2]+0.46):
		p.PID(current[0], current[1], final[2])
        	print('going down to the marker...')
		r.sleep()

    	x.activate_gripper = False
    	activate.wait_for_service()
    	activate.call(x.activate_gripper)
    	print('box is droped gently (becuase its not a bomb :D)')

    	# going up 
    	current =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    	final=[p.Y[i][0],p.Y[i][1],height]
    
    	while not (p.pos_current[2] >=final[2]):
		p.PID(current[0],current[1],final[2])
        	print('going up')
		r.sleep()
    
    	current = [p.pos_current[0] , p.pos_current[1] , p.pos_current[2]+4]

    	p.Return(current)
    	final=[p.U[0][0], p.U[0][1], height]
    
    	condition = p.travel(current,final)

        p.Tuning(current,final)
    	while not condition:
		p.PID(final[0],final[1],final[2])
        	condition = p.travel(current,final)
		print("going for return")
		r.sleep()
    	final=[p.pos_current[0], p.pos_current[1], p.U[0][2]]
    
    	while not p.gripper_act == "True":
        	p.PID(final[0],final[1],final[2]-0.5)
		print("location", p.U[0])
		print("current_pos", p.pos_current)
        	print('drone goes down until it is ready to pick up the box')
        	r.sleep()  

    	# activates the gripper 		
    	x = Gripper()
    	x.activate_gripper = True
    	activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper )
    	activate.wait_for_service()
    	activate.call(x.activate_gripper)
    	print('box is attached to the drone')
    
    	current =[p.pos_current[0],p.pos_current[1],height]
    	#p.Return(current)

    	while not (p.pos_current[2] >= current[2]):
        	p.PID(current[0] , current[1] , current[2])
        	print("going up")
        	r.sleep()
    
    	current = [p.pos_current[0] , p.pos_current[1] , p.pos_current[2]+4]
    	#p.Return(current)
    	final=[p.D[p.W[0]][0],p.D[p.W[0]][1],height]
    	condition = p.travel(current,final)

        p.Tuning(current,final)
    	while not condition :
		p.PID(final[0],final[1],height)
        	condition = p.travel(current,final)
		print("going to return grid")
		r.sleep()
    	final=[p.D[p.W[0]][0],p.D[p.W[0]][1],p.D[p.W[0]][2]]
    	current =[p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    	while not (p.pos_current[2] <= final[2]+0.26):
		p.PID(final[0], final[1], final[2])
        	print('going down to the grid')
		r.sleep()

    	x.activate_gripper = False
    	activate.wait_for_service()
    	activate.call(x.activate_gripper)
    	print('box is droped gently (becuase its not a bomb :D)')
        
        print('r num=',p.R_numbers)





		

       
