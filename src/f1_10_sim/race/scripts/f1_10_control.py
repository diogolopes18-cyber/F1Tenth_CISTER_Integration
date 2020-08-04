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
from time import strftime, gmtime
from os.path import expanduser
import datetime as dt
import csv

#distance_file=open('distance_file.txt','w')#Writes the positions obtained from LIDAR in a file

LIDAR_ODOMETRY_ERROR = 0

LOST_POSITION = 0
NORMAL_MODE = 1
mode = 1

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
PID_real_dist = 0.0
PID_dist = 0.0

kp_theta = 0.5
kd_theta = 0.0000
ki_theta = 0.000

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
leader_position_array = np.empty([1,3])#Stores the position of the leader in an array, in order to compare the position with the follower


########################
#FOLLOWER PARAMETERS
########################
latitude_follower_2 = 0.0
longitude_follower_2 = 0.0
heading_follower_2 = 0.0
speed_follower_2 = 0.0
orientation_x_car2 = 0.0
orientation_y_car2 = 0.0
steering_angle_car2 = 0.0

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
MAX_DISTANCE = 5.0
MIN_DISTANCE = 1.0

#CONVERT RADIAN TO DEGREES
DEGREE_CONVERSION = 180/(np.pi)

################
#TIME FLAGS
################

direction_control_time_flag = 0.0
time_control_flag = 0.0
MIN_TIME_STAMP = 0.1

###############
#ANGLES
###############
theta_error = 0.0
theta_error_old = 0.0
theta_pid_control = 0.0
theta_integral = 0.0
theta_derivative_value = 0.0
MAXIMUM_ANGLE_FOV = 45
MINIMUM_ANGLE_FOV = 2.0

###############
#CSV FILE
###############
#user_directory = expanduser('~/sims_ws')#Defines the path to home directory
#csv_file = open(strftime('Meausure_Comparison_%Y_%m_%d_%H_%M_%S',gmtime())+'.txt','w')#Opens file in home directory

# def csv_writting():

#     write_time = time.time()
#     file.write('%f,%f\n' % ())


#Atributing to the variable velocity the value of the msg file in order to be used througout the script
def car_parameters_leader(msg):

    global speed_leader
    global steering_angle

    speed_leader=msg.velocity
    steering_angle=msg.angle

def car_parameters_car2(msg):
    global speed_follower_2
    global steering_angle_car2

    speed_follower_2 = msg.velocity
    steering_angle_car2 = msg.angle



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
    
    #Latitude and longitude
    latitude_leader = data.pose.pose.position.x
    longitude_leader = data.pose.pose.position.y

    #platooning_control()


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

    general_control()


################################################################################################################
#Controls the position between the real position of leader car and the desired position
#Orientation gives us the desired trajectory towards a target
#In order to have a functioning platoon, we must target the orientation of car2 towards the postion of car1
################################################################################################################

def direction_control(lat_leader, long_leader,head_leader,head_follower):

    #global latitude_leader
    global latitude_follower_2
    #global longitude_leader
    global longitude_follower_2
    global orientation_x_car2
    global orientation_y_car2
    global velocity
    global speed_follower_2
    global speed_leader
    global direction_control_time_flag
    global theta_pid_control
    global theta_error_old
    global theta_error
    global theta_derivative_value

    speed_follower_2 = velocity#Stores the value of velocity to car2
    speed_leader = velocity


    program_time = time.time()

    time_delay = program_time - direction_control_time_flag

    #If the simulation is started, then the differences between longitude and latitude will be calculated
    if(time_delay >= MIN_TIME_STAMP):
        direction_control_time_flag = program_time#Saves time for next iteration

        diff_lat = lat_leader - latitude_follower_2
        diff_long = long_leader - longitude_follower_2

        #Since we have the desired values of latitude and longitude, we know where car1 must be at any time
        dist_to_leader = m.sqrt((diff_lat**2)+(diff_long**2))
        print("Distance to leader:", dist_to_leader)

        #Control for distance
        if(dist_to_leader <= MIN_DISTANCE):
            dist_to_leader == MIN_DISTANCE

        #Make car2 orientation to be car1 position
        orientation_x_car2 = lat_leader
        orientation_y_car2 = long_leader

        theta_error = (head_leader) - (head_follower)
        if(abs(theta_error) > MINIMUM_ANGLE_FOV):

            theta_derivative_value = theta_error_old - theta_error
            theta_pid_control = (kp_theta*theta_error + kd_theta*theta_derivative_value)

            if(theta_pid_control > MAX_STEERING_ANGLE):
                theta_pid_control = MAX_STEERING_ANGLE
            elif(theta_pid_control < -MAX_STEERING_ANGLE):
                theta_pid_control = -MAX_STEERING_ANGLE

        theta_error = 0
        theta_error_old = 0
        theta_pid_control = 0
        theta_integral = 0

        return (-theta_pid_control)
    
    else:
        return 0.0




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

    absurde_value=0

    measures_lidar=arr.array('f',[0])
    measures_lidar.extend(data.ranges)

    for i in range(len(data.ranges)):
        angle_index=(data.angle_min)+(i*(data.angle_increment))#Gives the angle to a specific point
        lidar_coordinates_x=data.ranges[i]*np.cos(angle_index)
        lidar_coordinates_y=data.ranges[i]*np.sin(angle_index)

        if(data.ranges[i]>MAX_DISTANCE):
            data.ranges[i] == MAX_DISTANCE



    #Calculates distance in all FOV 
    total_distance=m.sqrt((lidar_coordinates_x**2)+(lidar_coordinates_y**2))#Calculates distance through hypotenuse
    print("Total distance meausured by LiDAR\n", total_distance)#Distance in meters
    time.sleep(0.1)
    #print(data.ranges[320])

    
    
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

    while True:
        try:
            with open('distance_file.txt','w') as distance_file:
                distance_file.write(str(total_distance))#Writes distance into file
        except:
            print("Not able to write to file")


# def following_leader():

#     global leader_position_array
#     global latitude_leader
#     global longitude_leader
#     global speed_leader
#     global latitude_follower_2
#     global longitude_follower_2
#     global heading_follower_2
#     global mode

#     lost_distance = 0
#     dist = 0
#     dist_lost = 0
#     dist_lost_min = 999
#     pos_lost = 0
#     pos = 0
#     dist_min = 999

#     leader_position_array = np.append(leader_position_array,[[latitude_leader,longitude_leader,speed_leader]],axis=0)

#     for i in range(leader_position_array.shape[0]):
#         following_lat_diff = leader_position_array[i,0] - latitude_follower_2
#         following_long_diff = leader_position_array[i,0] - longitude_follower_2

#         angle_leader_follower = m.atan2(following_long_diff,following_lat_diff)#Calculates the angle between the positions of leader and follower

#         angle_diff = heading_follower_2 - angle_leader_follower#Difference between follower and the angle between leader and follower

#         if(abs(angle_diff) < MINIMUM_ANGLE_FOV):
#             print(1)
#             dist = m.sqrt((leader_position_array[i,1] - longitude_follower_2)**2 + (leader_position_array[i,0] - latitude_follower_2)**2)
#         else:
#             dist_lost = m.sqrt((leader_position_array[i,1] - longitude_follower_2)**2 + (leader_position_array[i,0] - latitude_follower_2)**2)

#             if dist_lost_min > dist_lost and dist_lost > 0.0:
#                 pos_lost = i
#                 dist_lost_min = dist_lost

        
#         if dist_min > dist and dist > 0.0:
#             pos = i
#             dist_min = dist             #else, update dist_min value
#             mode = NORMAL
#         elif dist_min < dist:
#             mode = NORMAL
#             break
#         elif (i == (TV_position_vector.shape[0] - 1)):
#             pos = pos_lost
#             mode = LOST

#     latitude_leader = TV_position_vector[pos,0]                 #update the value of TV_latitude 
#     longitude_leader = TV_position_vector[pos,1]                #update the value of TV_longitude 
#     #heading_out = TV_position_vector[pos,2]
#     speed_leader = TV_position_vector[pos,2]
#     TV_position_vector = TV_position_vector[pos:,:]     #clean the vector, excluding the old positions

#####################################################################
#Compare the latitude and longitude from LiDAR and Gazebo odometry
#####################################################################
def compare_meausures():

    global lidar_coordinates_x
    global lidar_coordinates_y
    global latitude_leader
    global longitude_leader

    #Calculate difference between meausurements from LiDAR and Gazebo
    latitude_leader_compare = latitude_leader - lidar_coordinates_x
    longitude_leader_compare = longitude_leader - lidar_coordinates_y

    execution_time = time.time()
    delay = execution_time - direction_control_time_flag

    #Check for errors and values that aren't numbers
    if(m.isnan(latitude_leader_compare) or m.isnan(longitude_leader_compare)):
        return LIDAR_ODOMETRY_ERROR
    else:
        latitude_leader = latitude_leader_compare
        longitude_leader = longitude_leader_compare
    
    print("Test:",latitude_leader_compare)

    if(delay >= MIN_TIME_STAMP):
        with open('compare.txt','w') as compare:
            compare.write(str(latitude_leader_compare,latitude_leader))#Writes distance into file

############################################################################
#General platooning control
#Receives data from steering angle and direction and makes corrections
#NEED TO ADD PID CONTROL
############################################################################

# def platooning_control():
#     car_control_msg = AckermannDriveStamped()

#     car_control_msg.drive.steering_angle = direction_control()

##########################################################################
#Need to add a main function where all the info from cars is processed
#Add PID control to this function
##########################################################################
def general_control():      #STILL NEED TO TEEST
    
    #Leader data
    global latitude_leader
    global longitude_leader
    global heading_leader
    global speed_leader
    #global steering_angle

    #Follower data
    global longitude_follower_2
    global latitude_follower_2
    global heading_follower_2
    global speed_follower_2

    #Timers
    global time_iteration_2
    global initial_time
    global time_flag

    #PID
    global PID_real_dist

    time_now = time.time()
    time_lapse = time_now - time_control_flag

    ctrl_msg_car2 = drive_param()

    if(time_lapse >= MIN_TIME_STAMP):

        platoon_distance = m.sqrt((latitude_leader - latitude_follower_2)**2 + (longitude_leader - longitude_follower_2)**2)
        print("Distance:", platoon_distance)

        platoon_distance_error = platoon_distance - MAX_DISTANCE
        print("Distance error:", platoon_distance_error)

        ctrl_msg_car2.angle = direction_control(latitude_leader,longitude_leader,heading_leader,heading_follower_2)

        
    # platoon_distance = m.sqrt(((longitude_leader - longitude_follower_2)**2) + ((latitude_leader - latitude_follower_2)**2))
    # platoon_distance_error = platoon_distance - MAX_DISTANCE
    # print(platoon_distance_error)

    # #if(platoon_distance > MIN_DISTANCE and speed_leader > MIN_SPEED):
            
    # PID_real_dist = longitudinal_control(platoon_distance_error)
    # print(21)

    # steering_angle = direction_control(latitude_leader,longitude_leader,heading_leader,heading_follower_2)

def listener():
    print("F1/10 node started")
    rospy.init_node('f1_10', anonymous=True)
    rospy.Subscriber('/drive_parameters', drive_param, car_parameters_leader)#Subscribes to topic that stores velocity and steering angle
    rospy.Subscriber('/drive_parameters/car2', drive_param, car_parameters_car2)
    rospy.Subscriber('/scan/car2', LaserScan, lidar_meausurements)
    rospy.Subscriber('/car2/odom', Odometry, car2_info)
    rospy.Subscriber('/car1/odom', Odometry, car1_info)
    rospy.spin()

if __name__ == '__main__':
    listener()