#!/usr/bin/env python

import numpy as np
import array as arr
import rospy
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from race.msg import drive_param
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion
import tf
import math as m
import time

car_position=np.empty([1,4])#Creates an array with 3 positions
#distance_file=open('distance_file.txt','w')#Writes the positions obtained from LIDAR in a file

############################################
# PID CONTROL USING ZIEGLER NICHOLS METHOD
############################################

angle_range=180

velocity = 0.0
steering_angle = 0.0
leader_name = 'car1'
follower_name='car2'
heading_leader = 0.0

def obtain_position(data):

    #data=Odometry()

    quaternion = (					#the orientation is published as a quaternion vector.
	    data.pose.pose.orientation.x,
	    data.pose.pose.orientation.y,
	    data.pose.pose.orientation.z,
	    data.pose.pose.orientation.w)
    euler_tf=tf.transformations.euler_from_quaternion(quaternion)

    #Conversion from quaternion coordinates to euler transforms
    #roll=euler_tf[0]
    #pitch=euler_tf[1]
    yaw=euler_tf[2]

    #Checks for not valid number formats
    try:
        if(m.isnan(yaw)==False):
            pass
    except:
        print("Not a number")
    
    #Converts to degrees
    heading_leader=m.degrees(yaw)

    print(heading_leader)
    
    # if(heading_leader>1):
    #     print("Teste"+'\n')
    # else:
    #     print("Not able to collect data")
    




#Atributing to the variable velocity the value of the msg file in order to be used througout the script
def car_velocity(msg):

    global velocity
    global steering_angle

    velocity=msg.velocity
    steering_angle=msg.angle


##########################
# LIDAR MEASUREMENTS
##########################
def lidar_meausurements(data):

    global lidar_coordinates_x
    global lidar_coordinates_y
    global measures_lidar
    global total_distance
    global velocity
    global steering_angle
    # global index
    # global theta

    #theta=180

    absurde_value=0

    measures_lidar=arr.array('f',[0])
    measures_lidar.extend(data.ranges)

    for i in range(len(data.ranges)):
        angle_index=(data.angle_min)+(i*(data.angle_increment))#Gives the angle to a specific point
        lidar_coordinates_x=data.ranges[i]*np.cos(angle_index)
        lidar_coordinates_y=data.ranges[i]*np.sin(angle_index)
        #print("LIDAR Coordinates in X Axis:", lidar_coordinates_x)
        #print("LIDAR Coordinates in Y Axis:", lidar_coordinates_y)

    #Calculates distance in all FOV 
    total_distance=m.sqrt((lidar_coordinates_x**2)+(lidar_coordinates_y**2))#Calculates distance through hypotenuse
    print("Total distance\n", total_distance)#Distance in meters
    time.sleep(0.5)
    
    
    #Prints only the front distance of the FOV
    #print(data.ranges[360])

    # if theta > 179.9:
    #     theta = 179.9

    # index = len(data.ranges)*theta/angle_range
    # total_distance = data.ranges[int(index)]
    # if m.isinf(total_distance) or m.isnan(total_distance):
    #     return 4.0
    # print(data.ranges[int(index)])

    #Checks for infinite numbers or something that isn't a number
    if(m.isinf(total_distance) or m.isnan(total_distance)):
        return absurde_value

    try:
        with open('distance_file.txt','w') as distance_file:
            distance_file.write(str(total_distance))#Writes distance into file
    except:
        print("Not able to write to file")
    
    if(total_distance>1.2 or total_distance<2.0):
        velocity=1.5
        steering_angle=0.0
    elif total_distance<1:
        velocity=2.0





def listener():
    print("F1/10 node started")
    rospy.init_node('f1_10', anonymous=True)
    rospy.Subscriber('/drive_parameters', drive_param, car_velocity)#Subscribes to topic that stores velocity and steering angle
    rospy.Subscriber('/scan', LaserScan, lidar_meausurements)
    rospy.Subscriber('/car1/odom', Odometry, obtain_position)
    rospy.spin()

if __name__ == '__main__':
    listener()