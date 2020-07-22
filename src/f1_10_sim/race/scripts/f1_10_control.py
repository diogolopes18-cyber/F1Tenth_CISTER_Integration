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
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tf
import math as m
import time
import datetime as dt

car_position=np.empty([1,4])#Creates an array with 3 positions
#distance_file=open('distance_file.txt','w')#Writes the positions obtained from LIDAR in a file

LIDAR_ODOMETRY_ERROR = 0

###########
#PLOT
###########
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

############################################
# PID CONTROL USING ZIEGLER NICHOLS METHOD
############################################

angle_range=180

####################################
#Velocity and steering parameters
####################################
velocity = 0.0      #Global variable
steering_angle = 0.0
leader_name = 'car1'
follower_name = 'car2'



######################
#LEADER PARAMETERS
######################
latitude_leader = 0.0
longitude_leader = 0.0
heading_leader = 0.0    #Global variable
speed_leader = 0.0



########################
#FOLLOWER PARAMETERS
########################
latitude_follower_2 = 0.0
longitude_follower_2 = 0.0
heading_follower_2 = 0.0
speed_follower_2 = 0.0
orientation_x_car2 = 0.0
orientation_y_car2 = 0.0

########################################################################################
#CORRECTED PARAMETERS BASED ON THE COMPARISON ON THE ODOMETRY FROM LiDAR AND GAZEBO
########################################################################################
latitude_compare_car_2 = 0.0
longitude_compare_car_2 = 0.0
orientation_compare_x_car_2 = 0.0
orientation_compare_y_car_2 = 0.0
latitude_leader_compare = 0.0
longitude_leader_compare = 0.0
corrected_distance_lidar = 0.0  #Distance with the corrected value after comparison between LiDAR and Gazebo odometry

#######################
#LiDAR PARAMETERS
#######################
lidar_coordinates_x = 0.0
lidar_coordinates_y = 0.0
measures_lidar = 0.0
total_distance = 0.0

###############################
#SPEED AND DIRECTION CONTROL
###############################
MAX_STEERING_ANGLE = 1.0
MAX_SPEED = 2.5

#CONVERT RADIAN TO DEGREES
DEGREE_CONVERSION = 180/(np.pi)

################
#TIME FLAGS
################

direction_control_time_flag = 0.0
MIN_TIME_STAMP = 0.1


#Atributing to the variable velocity the value of the msg file in order to be used througout the script
def car_velocity(msg):

    global velocity
    global steering_angle

    velocity=msg.velocity
    steering_angle=msg.angle



#################################################
#Traces plot for distance in time in real time
#################################################

def animate(i, xs, ys):

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(velocity)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Distance to leader over time')
    plt.ylabel('Distance(m)')

#######################################################
#Obtains position data for car1
#######################################################

def car1_info(data):
    
    msg=AckermannDriveStamped()

    global heading_leader
    global latitude_leader
    global longitude_leader

    #Control conditions
    if(msg.drive.steering_angle > MAX_STEERING_ANGLE):
        msg.drive.steering_angle == MAX_STEERING_ANGLE
    
    if(msg.drive.speed > MAX_SPEED):
        msg.drive.speed == MAX_SPEED
    
    #Latitude and longitude
    latitude_leader = data.pose.pose.position.x
    longitude_leader = data.pose.pose.position.y

    platooning_control()


#######################################################
#Obtains position and orientation data for car2
#######################################################

def car2_info(data):

    global heading_follower_2
    global orientation_x_car2
    global orientation_y_car2
    global latitude_follower_2
    global longitude_follower_2
    
    msg = AckermannDriveStamped()

    quaternion = (					#the orientation is published as a quaternion vector.
	    data.pose.pose.orientation.x,
	    data.pose.pose.orientation.y,
	    data.pose.pose.orientation.z,
	    data.pose.pose.orientation.w)
    euler_tf=tf.transformations.euler_from_quaternion(quaternion)

        #Control conditions
    if(msg.drive.steering_angle > MAX_STEERING_ANGLE):
        msg.drive.steering_angle == MAX_STEERING_ANGLE
    
    if(msg.drive.speed > MAX_SPEED):
        msg.drive.speed == MAX_SPEED

    x_position=data.pose.pose.position.x
    #roll=euler_tf[0]
    #pitch=euler_tf[1]
    yaw=euler_tf[2]

    #Longitude and Latitude
    latitude_follower_2 = data.pose.pose.position.x
    longitude_follower_2 = data.pose.pose.position.y

    #Orientation for car2
    orientation_x_car2 = data.pose.pose.orientation.x
    orientation_y_car2 = data.pose.pose.orientation.y

    #Checks for not valid number formats
    try:
        if(m.isnan(yaw)==False):
            pass
    except:
        print("Not a number")
    
    #Converts to degrees
    heading_follower_2= yaw * DEGREE_CONVERSION


################################################################################################################
#Controls the position between the real position of leader car and the desired position
#Orientation gives us the desired trajectory towards a target
#In order to have a functioning platoon, we must target the orientation of car2 towards the postion of car1
################################################################################################################

def direction_control():

    global latitude_leader
    global latitude_follower_2
    global longitude_leader
    global longitude_follower_2
    global orientation_x_car2
    global orientation_y_car2
    global velocity
    global speed_follower_2
    global speed_leader

    speed_follower_2 = velocity#Stores the value of velocity to car2
    speed_leader = velocity


    program_time = time.time()

    time_delay = program_time - direction_control_time_flag

    #If the simulation is started, then the differences between longitude and latitude will be calculated
    if(time_delay >= MIN_TIME_STAMP):
        diff_lat = latitude_leader - latitude_follower_2
        diff_long = longitude_leader - longitude_follower_2

        #Since we have the desired values of latitude and longitude, we know where car1 must be at any time
        dist_to_leader = m.sqrt((diff_lat**2)+(diff_long**2))
        print("Distance to leader:", dist_to_leader)

        #Make car2 orientation to be car1 position
        orientation_x_car2 = latitude_leader
        orientation_y_car2 = longitude_leader

        if(dist_to_leader < 0.5):#Checks distance in order to stop, if leader stops as well
            if(speed_leader == 0.0 or speed_leader < 0.2):#If the leader velocity is zero or very low, the follower will stop as well
                speed_follower_2 = 0.0

        # plt.plot([program_time,dist_to_leader])
        # plt.show()
    #Create an array to store the distance to leader read by LIDAR and compare the distance read from LIDAR and the one published by /car2/odom
    #Read distance to leader from LIDAR
    #Calculate difference from distance to leader from /car2/odom and LIDAR
    #Store the position of every distance read by LIDAR in an array in order to use in mapping



# Set up plot to call animate() function periodically
# ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
# plt.show()


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
    time.sleep(0.1)
    
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
    
    # if(total_distance>1.2 or total_distance<2.0):
    #     velocity=1.5
    #     steering_angle=0.0
    # elif total_distance<1:
    #     velocity=2.0


#####################################################################
#Compare the latitude and longitude from LiDAR and Gazebo odometry
#####################################################################
def compare_meausures():

    global lidar_coordinates_x
    global lidar_coordinates_y
    global latitude_leader
    global longitude_leader

    #Calculate difference between meausurements from LiDAR and Gazebo
    latitude_leader_compare = lidar_coordinates_x - latitude_leader
    longitude_leader_compare = lidar_coordinates_y - longitude_leader

    #Check for errors and values that aren't numbers
    # if(m.isnan(latitude_leader_compare) or m.isnan(longitude_leader_compare)):
    #     return LIDAR_ODOMETRY_ERROR
    # else:
    #     latitude_leader = latitude_leader_compare
    #     longitude_leader = longitude_leader_compare


############################################################################
#General platooning control
#Receives data from steering angle and direction and makes corrections
#NEED TO ADD PID CONTROL
############################################################################

def platooning_control():
    car_control_msg = AckermannDriveStamped()

    car_control_msg.drive.steering_angle = direction_control()

##########################################################################
#Need to add a main function where all the info from cars is processed
#Add PID control to this function
##########################################################################

def listener():
    print("F1/10 node started")
    rospy.init_node('f1_10', anonymous=True)
    rospy.Subscriber('/drive_parameters', drive_param, car_velocity)#Subscribes to topic that stores velocity and steering angle
    rospy.Subscriber('/scan', LaserScan, lidar_meausurements)
    rospy.Subscriber('/car2/odom', Odometry, car2_info)
    rospy.Subscriber('/car1/odom', Odometry, car1_info)
    rospy.spin()

if __name__ == '__main__':
    listener()