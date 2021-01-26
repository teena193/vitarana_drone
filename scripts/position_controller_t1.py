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
       self.K_d = [ 5400*100000 ,5000*100000, 5000*2]
       
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
       self.l = msg.l
         
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
       self.ref_value = 5 #meters 
       self.error_lat = self.left - self.ref_value
       
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

   def PID_e(self , current_location , building_location):
       b,a = self.find_marker(building_location)
       ref_err = [0 , 0] 
       print("next pid")
      
       self.x_e = a - ref_err[0]
       self.y_e = b - ref_err[1]
       latitude = 5*self.x_e  
       longitude = 5*self.y_e
       print(latitude,longitude)
       
       self.PID(p.pos_current[0] - latitude , p.pos_current[1] +longitude , current_location[2])
       print("s",self.x_e,self.y_e)
      
     
       
   def travel(self , current , destination):

       #destination = [x1 , y1 , z1]
       #current = [x0 , y0 , z0]
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





if __name__ == '__main__':

    p = Position()
    r = rospy.Rate(30)
    A_1 = [18.9999864489,71.9999430161,8.44099749139]
    B_2 = [18.9999864489 + p.x_to_lat(1.5),71.9999430161 - p.y_to_long(1.5),8.44099749139]#[19.0000000000062, 71.99995726219537, 8.44099749139]
    C_1 = [18.9999864489+p.x_to_lat(3),71.9999430161,8.44099749139]    #19.0004681325,72.0000949773,16.660019864
    
    initial_location = [p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    final_location = [A_1[0] , A_1[1] ,A_1[2]+1]
    #while not (p.pos_current[0] <= A_1[0] and p.pos_current[1] <=A_1[1] and p.pos_current[2]>= A_1[2] + 1):
        #p.PID(A_1[0],A_1[1],A_1[2]+1)
        
        #r.sleep()

    condition = p.travel(initial_location , final_location)
    print(condition)

    while not condition:
	p.PID(A_1[0],A_1[1],A_1[2]+1)
	r.sleep()
	condition = p.travel(initial_location , final_location)
	print(condition)
        
    

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

    initial_location = [p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    final_location = [p.data[0][1], p.data[0][2], p.data[0][3]+1]

    condition = p.travel(initial_location , final_location)
  
    while not condition:
        p.PID(p.data[0][1],p.data[0][2],p.data[0][3] + 1)
	r.sleep()
	condition = p.travel(initial_location , final_location)
	print(condition)
        


    i = 1
    while p.l == 0:
	
	p.PID(p.data[0][1],p.data[0][2],p.data[0][3] + 1 + 2.5*i)
	r.sleep()
	if p.pos_current[2] >= p.data[0][3] + 1 + 2.5*i :
		i =i+1


    initial_location = [p.pos_current[0],p.pos_current[1],p.pos_current[2]]
    b,a = p.find_marker(p.data[0][3])
    current_location = [p.pos_current[0] -a ,p.pos_current[1]+b ,p.pos_current[2]] 
    condition = p.travel(initial_location , current_location)
    
    while not condition:
	print("going towards marker")
	p.PID(current_location[0] - a,current_location[1] + b,current_location[2])
	r.sleep()
	condition = p.travel(initial_location , current_location)
	print(condition)
        


    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    final_location = [p.data[0][1], p.data[0][2], p.data[0][3]]
    condition = p.travel()

    while not (p.pos_current[2]<=p.data[0][3]+0.4):
        p.PID(current_location[0],current_location[1],p.data[0][3])
        print("last")
        r.sleep()
    x.activate_gripper = False
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is droped gently (becuase its not a bomb :D)')
    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    while (p.bottom_range<=2):
        p.PID(C_1[0],C_1[1],current_location[2] +1)
        print("moving")
        r.sleep()
    print(B_2)
    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]]
    print(current_location)
    while not (p.linear_acc_x <= 0.0002 and p.linear_acc_y <= 0.0002 and p.linear_acc_z <= 0.0002 and p.vel_x <=0.05 and p.vel_y<=0.05 and p.vel_z <= 0.05):
        p.PID(C_1[0],C_1[1],C_1[2] +1)
        print("moving to box 2")
        r.sleep()
        
    while not p.gripper_act == "True":
        p.PID(C_1[0],C_1[1],C_1[2])
        r.sleep()  
    
    x = Gripper()
    x.activate_gripper = True
    activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper )
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is attached to the drone') 

    while  not (p.pos_current[0]>=p.data[1][1] and p.pos_current[1]>=p.data[1][2] and p.pos_current[2]>=p.data[1][3]):
           
         p.PID(p.data[1][1],p.data[1][2],p.data[1][3] + 1)
         r.sleep()
    while p.l == 0:
	print("searching")
	p.PID(p.data[1][1],p.data[1][2],p.data[1][3] + 1 + 1*i)
	r.sleep()
	if p.pos_current[2] >= p.data[1][3] + 1 + 1*i :
		i =i+1
    
    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]] 
    b,a = p.find_marker(p.data[1][3])
    while not (p.pos_current[0]<=current_location[0]-a and p.pos_current[1]>=current_location[1] + b):
        
        
	print("building C_1")
	p.PID(current_location[0] - a,current_location[1] + b,current_location[2])
        r.sleep()
    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]]
    while not (p.pos_current[2]<=p.data[1][3]+0.4):
        p.PID(current_location[0],current_location[1],p.data[1][3])
        print("last")
        r.sleep()
    x.activate_gripper = False
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is droped gently (becuase its not a bomb :D)')
    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]]
    while (p.bottom_range<=2):
        print(p.bottom_range)
        p.PID(B_2[0],B_2[1],current_location[2] +1)
        print("moving")
        r.sleep()
    print(B_2)
    current_location = [p.pos_current[0] ,p.pos_current[1] ,p.pos_current[2]]
    print(current_location)
    while not (p.linear_acc_x <= 0.0002 and p.linear_acc_y <= 0.0002 and p.linear_acc_z <= 0.0002 and p.vel_x <=0.05 and p.vel_y<=0.05 and p.vel_z <= 0.05):
        p.PID(B_2[0],B_2[1],B_2[2] +1)
        print("moving to box 3")
        r.sleep()
        
    while not p.gripper_act == "True":
        p.PID(B_2[0],B_2[1],B_2[2])
        r.sleep()  
    
    x = Gripper()
    x.activate_gripper = True
    activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper )
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is attached to the drone')
    
    

    
    
        
    
       
  
	

     
   
   




 

		

       
