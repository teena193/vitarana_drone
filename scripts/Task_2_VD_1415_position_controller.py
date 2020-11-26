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
       self.K_p = [ 5000*1000,  5000*1000, 4000*0.1]
       self.K_i = [    7*0.1,      7*0.1, 950*0.001]
       self.K_d = [ 3188*200000 ,3188*200000, 5000*2]
       
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
      
       #publishers
       self.cmd_publish = rospy.Publisher('/drone_command',edrone_cmd,queue_size = 1)

       #subcription
       rospy.Subscriber('/edrone/gps',NavSatFix,self.Nav_data)
       rospy.Subscriber("/edrone/location" , NavSatFix , self.final_dest) 
       rospy.Subscriber('/edrone/gripper_check' , String , self.gripper)
       rospy.Subscriber('/edrone/imu/data', Imu, self.distance)
       rospy.Subscriber('/edrone/range_finder_top' , LaserScan , self.Ranges)
       rospy.Subscriber('/edrone/gps_velocity' , Vector3Stamped , self.Velocity)

   #defining callback_function
   
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
       if (self.left < 10):
           #we put this self.r variable true so if it becomes true, we can indicate to our path planner that it needs to create a new path after the obstacle is avoided 
	   self.r = True
 	   self.stop_loc = [self.pos_current[0], self.pos_current[1], self.pos_current[2]]
           # as long as the drone is not stable it will try to stop it by using the function Stop() defined below, and to check if it stable we use data from imu linear acceration and gps velocity.
	   while not (self.linear_acc_x <= 0.0002 and self.linear_acc_y <= 0.0002 and self.linear_acc_z <= 0.0002 and self.vel_x <=0.05 and self.vel_y<=0.05 and self.vel_z <= 0.05):

               self.Stop()
               print('Stopping')
               r.sleep()
           # while the condtion is True our drone will avoid the obstacle by using a command (here it is backward (according to the drone's orientation))
           while (self.left < 12):

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




if __name__ == '__main__':

    p = Position()
    r = rospy.Rate(30)
    #the location of the box
    box_location = [19.0007046575  ,71.9998955286 ,22.1599967919] 
   
    # j and k is to indicates the no. of waypoint
    j = 1
    k = 1

    # going 1.2m up from its initial latitude
    while not (p.pos_current[2] >= box_location[2] + p.height_req_for_qr):
    	p.PID(p.pos_current[0], p.pos_current[1], box_location[2] + p.height_req_for_qr)     
        r.sleep()
        print("going up...")

    #path between initial location and box location is planned 
    p.Path_Planning(p.initial, box_location)

    #uses the planned waypoints to navigate to the box
    while  (j < (p.way_points_no + 2)):
        p.PID(p.way_points_latitude[j], p.way_points_longitude[j], box_location[2]+ p.height_req_for_qr)
        r.sleep()
        print("traveling to the box location...")
                                                                                              
        if (p.pos_current[0] <= p.way_points_latitude[j]) and (p.pos_current[1] <= p.way_points_longitude[j]):  
            j+=1   

    #goes down to box location to pick up the box    
    while not (p.gripper_act == "True"):
	p.PID(box_location[0], box_location[1], box_location[2])
	r.sleep()
        print("going down to pick up the box...")

    # activates the gripper 		
    x = Gripper()
    x.activate_gripper = True
    activate = rospy.ServiceProxy('/edrone/activate_gripper' , Gripper ) 
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is attached to the drone')

    #takes the box  above the box location 	
    while not (p.pos_current[2] >= (box_location[2] + p.height_req_for_qr)) :	
	p.PID(box_location[0], box_location[1], box_location[2] + p.height_req_for_qr)
        r.sleep()
        print('going up ...')

    #plans a path from box to the location provided by the qr code 	
    p.Path_Planning(box_location, p.final_loc)
      
    #uses the planned paths to navigate to the final location along with obstacle avoidance 	
    while  (k < (p.way_points_no + 2)):
        p.PID(p.way_points_latitude[k], p.way_points_longitude[k], box_location[2]+ 1.2)
        p.Obstacle_avoid()
        r.sleep()
        print('navigating to the final location')
        
        #if obstacle is found our drone avoids it and then plans a new path  
        if (p.r == True):
	    p.Path_Planning(p.pos_after_obs, p.final_loc)
	    k = 1
            p.r = False
        #condition so that the drone starts to move to the next waypoint when the last one is achieved
        if (p.pos_current[0] <= p.way_points_latitude[k] ) and (p.pos_current[1] >= p.way_points_longitude[k]):
            k+=1  
            
    #the drone drops its height for it to be able to drop the box (gently)
    while not(p.pos_current[2] <= p.final_loc[2]+p.box_height):
	p.PID(p.final_loc[0], p.final_loc[1], p.final_loc[2])
	print("going down to the final location...")
	r.sleep()
    # the gripper is deactivated	
    x.activate_gripper = False
    activate.wait_for_service()
    activate.call(x.activate_gripper)
    print('box is droped gently (becuase its not a bomb :D)')
    
    # the drone lifts up after droping the box gently( because its not a bomb :D ) 
    while not rospy.is_shutdown():
    	p.PID(p.final_loc[0], p.final_loc[1] ,p.final_loc[2]+1.2)	
	r.sleep()
        print('drone lifts up')
    	





 

		

       
