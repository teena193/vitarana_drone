#!/usr/bin/env python


# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import Imu
import math 
import rospy
import time
import tf


class Edrone():
    def __init__(self):

        rospy.init_node('attitude_controller',anonymous=True)  # initializing ros node with name drone_control

        # This corresponds to our current orientation of eDrone in quaternion format. This value is updated each time in our imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to our current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500, 1500, 1500, 1000]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        #setting of Kp, Kd and ki for [roll, pitch, yaw].
        self.Kp = [82*0.01, 175*0.01, 1223*0.1]
        self.Ki = [0*0.001, 0, 96*0.001]
        self.Kd = [52*0.2, 153*0.2, 0*2]
        
        
        # previous values of error for differential part of PID
        # [Roll error previous, Pitch error previous, Yaw error previous]
        self.prev_values_error = [0.0, 0.0, 0.0]
       
        # Maximum and Minimum value of propeller speed.
        # [Propeller 1, propeller 2, propeller 3, propeller 4]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]
         
        # Error terms for PID
        # [Roll error, Pitch error, Yaw error]
        self.error = [0.0, 0.0, 0.0]
     
        # Integral terms for PID
        # [Roll integral term, Pitch integral term, Yaw integral term]
        self.iterm = [0.0, 0.0, 0.0]
    
        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 30

        # Publishing /edrone/pwm
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
  
        
        # Subscribing to /drone_command, imu/data
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
      
    # Callback function for Imu, where we store the data from IMU 
    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x 
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        
    
    # Callback function for e_drone_cmd , where we get the data from the position controller script. 
    def drone_command_callback(self, msg):

        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle
       

    def pid(self):

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[1],self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
         
        
        # Convertng the range of rcRoll ,rcPitch and rcYaw from 1000 - 2000 to -10 degree to 10 degree
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30
        
        # Calculating the error terms 
        self.error[0] = math.degrees(self.drone_orientation_euler[0]) - self.setpoint_euler[0]
        self.error[1] = math.degrees(self.drone_orientation_euler[1]) - self.setpoint_euler[1] 
        self.error[2] = math.degrees(self.drone_orientation_euler[2]) - self.setpoint_euler[2]
                     
  
        # Converting the throttle from the range of 1000 to 2000 to 0 - 1024
        Throttle = self.setpoint_cmd[3]*1.024 -1024
        
        # calculating Value for Roll
        self.iterm[0] = (self.iterm[0] + self.error[0]) * self.Ki[0] 
        Roll = (self.Kp[0] * self.error[0]) + (self.iterm[0]) + self.Kd[0] * (self.error[0] - self.prev_values_error[0])

        # Calculating value for Pitch
        self.iterm[1] = (self.iterm[1] + self.error[1]) * self.Ki[1]
        Pitch = (self.Kp[1] * self.error[1]) + (self.iterm[1]) + self.Kd[1] * (self.error[1] - self.prev_values_error[1])

        # Calculating value for Yaw
        self.iterm[2] = (self.iterm[2] + self.error[2]) * self.Ki[2]
        Yaw = (self.Kp[2] * self.error[2]) + (self.iterm[2]) + self.Kd[2] * (self.error[2] - self.prev_values_error[2])

       
        # Motor mixing algorithm  

        # front right 
        self.pwm_cmd.prop1 = Throttle + Roll + Yaw + Pitch  

        # back right 
        self.pwm_cmd.prop2 = Throttle + Roll - Yaw + Pitch  

        # back left
        self.pwm_cmd.prop3 = Throttle - Roll + Yaw + Pitch  

        # front left
        self.pwm_cmd.prop4 = Throttle - Roll - Yaw - Pitch  

       
        # To limit the maximum propeller speed to 1024
        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]

        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]

        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]

        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]

        # To limit the minimum propeller speed to 0 
        if self.pwm_cmd.prop1 < self.min_values[0]:
            self.pwm_cmd.prop1 = self.min_values[0]

        if self.pwm_cmd.prop2 < self.min_values[1]:
            self.pwm_cmd.prop2 = self.min_values[1]

        if self.pwm_cmd.prop3 < self.min_values[2]:
            self.pwm_cmd.prop3 = self.min_values[2]

        if self.pwm_cmd.prop4 < self.min_values[3]:
            self.pwm_cmd.prop4 = self.min_values[3]
       
        # Publishing the propeller speed
        self.pwm_pub.publish(self.pwm_cmd)
               
        # Storing the value of error to be used in the next iteration 
        self.prev_values_error[0] = self.error[0]
        self.prev_values_error[1] = self.error[1]
        self.prev_values_error[2] = self.error[2]



if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
