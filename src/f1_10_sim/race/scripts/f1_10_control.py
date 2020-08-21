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

kp_dis = 5.0
ki_dis = 0
kd_dis = 0.005

distance_integral = 0.0
distance_derivative_value = 0.0
distance_error_old = 0.0
distance_pid_control = 0.0

distance_pid = 0.0

####################################
#Velocity and steering parameters
####################################
velocity = 0.0      #Global variable
steering_angle = 0.0
leader_name = 'car1'
follower_name = 'car2'
speed_new_val = 0.0



######################
#LEADER PARAMETERS
######################
latitude_leader = 0.0
longitude_leader = 0.0
heading_leader = 0.0    #Global variable
speed_leader = 0.0
TV_position_vector = np.empty([1,4])#Stores the position of the leader in an array, in order to compare the position with the follower
latitude_leader_compare = 0.0#Compare measurements from LiDAR and Gazebo
longitude_leader_compare = 0.0


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
follower_position_array = np.empty([1,4])#Stores the positions of the follower

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
cross_track_distance = 0.0
lidar_sum = []
lidar_real_dist = 0.0

###############################
#SPEED AND DIRECTION CONTROL
###############################
MAX_STEERING_ANGLE = 30
MAX_SPEED = 2.5
MAX_DISTANCE = 5.0
MIN_DISTANCE = 1.0
MIN_SPEED = 0.1
MIN_LAT_DIST = 1.5

#CONVERT RADIAN TO DEGREES
DEGREE_CONVERSION = 180/(np.pi)

################
#TIME FLAGS
################

direction_control_time_flag = 0.0
time_control_flag = 0.0
MIN_TIME_STAMP = 0.01
lateral_control_time_flag = 0.0

###############
#ANGLES
###############
theta_error = 0.0
theta_error_old = 0.0
theta_pid_control = 0.0
theta_integral = 0.0
theta_derivative_value = 0.0
MAXIMUM_ANGLE_FOV = 45
MINIMUM_ANGLE_FOV = 0.005
ANGLE_ADJUST = m.pi/2

#####################
#STANLEY CONTROLLER
#####################
ke_t = 0.0
vf_t = 0.0
delta_final = 0.0
heading_correction = 0.0
path_heading = 0.0

###############
#CSV FILE
###############
#user_directory = expanduser('~/sims_ws')#Defines the path to home directory
csv_file = open('platoon_test.csv','w')#Opens file in home directory
lidar_file = open('lidar_comp.csv','w')

#Atributing to the variable velocity the value of the msg file in order to be used througout the script
def car_parameters_leader(msg):

    global speed_leader
    global steering_angle

    speed_leader=msg.velocity
    steering_angle=msg.angle

# def car_parameters_car2(msg):
#     global speed_follower_2
#     global steering_angle_car2

#     speed_follower_2 = msg.velocity
#     steering_angle_car2 = msg.angle



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
    plt.title('Distance to leader over time without control script')
    plt.ylabel('Distance(m)')


#########################################################
#Functions responsible for writting data into CSV files
#########################################################
def csv_file_write():
    
    global platoon_distance

    timestamp = time.time()
    timestamp_date = time.ctime(timestamp)
    csv_file.write('%s,%f\n' % (timestamp_date,platoon_distance))#Plots distance to leader to csv

def lidar_csv_write():

    global latitude_leader
    global latitude_leader_compare
    #global total_distance
    global lidar_real_dist
    global PID_real_dist

    timestamp = time.time()
    timestamp_date = time.ctime(timestamp)
    lidar_file.write('%s,%f,%f,%f\n' % (timestamp_date,lidar_real_dist,latitude_leader_compare, PID_real_dist))#Plots distance to leader to csv


#######################################################
#Obtains position data for car1
#######################################################

def car1_info(data):
    
    msg=AckermannDriveStamped()

    global heading_leader
    global latitude_leader
    global longitude_leader

    quaternion = (					#the orientation is published as a quaternion vector.
	    data.pose.pose.orientation.x,
	    data.pose.pose.orientation.y,
	    data.pose.pose.orientation.z,
	    data.pose.pose.orientation.w)
    euler_tf=tf.transformations.euler_from_quaternion(quaternion)

    yaw_leader = euler_tf[2]
    print("Yaw leader:",yaw_leader)
    heading_leader = yaw_leader * DEGREE_CONVERSION
    print("Heading leader:",heading_leader)
    
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

    x_position=data.pose.pose.position.x
    #roll=euler_tf[0]
    #pitch=euler_tf[1]
    yaw=euler_tf[2]
    print("Yaw 2:", yaw)

    #Longitude and Latitude
    latitude_follower_2 = data.pose.pose.position.x
    longitude_follower_2 = data.pose.pose.position.y

    print("Follower:", longitude_follower_2)
    print("Latitude:", latitude_follower_2)
    #print("Leader:", longitude_leader)

    #Orientation for car2
    orientation_x_car2 = data.pose.pose.orientation.x
    orientation_y_car2 = data.pose.pose.orientation.y

    # error_long = longitude_leader - longitude_follower_2
    # print("Test:", error_long)

    #Checks for not valid number formats
    try:
        if(m.isnan(yaw)==False):
            pass
    except:
        print("Not a number")
    
    #Converts to degrees
    heading_follower_2= yaw * DEGREE_CONVERSION
    print("Heading 2:",heading_follower_2)

    general_control()


###########################################################
#Controls the distance between the follower and leader
###########################################################
def longitudinal_control(error_distance):

    global direction_control_time_flag
    global distance_derivative_value
    global distance_error_old
    global distance_pid_control
    global distance_integral

    actual_time = time.time()
    time_diff = actual_time - direction_control_time_flag

    if(time_diff >= MIN_TIME_STAMP):
        direction_control_time_flag = actual_time#Saves current time for next iteration

        print("Test_1")#Debug control

        if(time_diff < 1):
            distance_derivative_value = distance_error_old - error_distance
        else:
            distance_derivative_value = distance_error_old - error_distance
        distance_error_old = error_distance#Updates the error value of distance with the value passed by the meausures from Gazebo odometry

        distance_pid_control = kp_dis * error_distance + kd_dis *distance_derivative_value
    
    else:
        distance_pid_control = 0.0
    
    return distance_pid_control#Returns the adjusted value from PID control of the distance between the follower and leader


# Set up plot to call animate() function periodically
# ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
# plt.show()


################################################################################################################
#Controls the position between the real position of leader car and the desired position
#Orientation gives us the desired trajectory towards a target
#In order to have a functioning platoon, we must target the orientation of car2 towards the postion of car1
################################################################################################################

def lateral_control(lat_leader, long_leader, head_leader, head_follower):

    # global leader_position_array
    # global latitude_follower_2
    # global longitude_follower_2
    # global follower_position_array
    global lateral_control_time_flag
    global theta_error_old
    global theta_error
    global theta_derivative_value
    global theta_integral
    global theta_pid_control

    sim_time = time.time()
    sim_delay = sim_time - lateral_control_time_flag

    if(sim_delay >= MIN_TIME_STAMP):
        lateral_control_time_flag = sim_time#Saves time for next iteration

        head_leader = head_leader - ANGLE_ADJUST
        head_follower = head_follower - ANGLE_ADJUST

        if (head_leader < - m.pi/4):#adjusting h_tv
            theta_TV = 2*m.pi + head_leader#adding 2pi in order to change for positive
        else:
            theta_TV = head_leader#if not, keep the angle

        if (head_follower < - m.pi/4):#adjusting h_sv
            theta_SV = 2*m.pi + head_follower#adding 2pi in order to change for positive
        else:
            theta_SV = head_follower#if not, keep the angle

        latitude_diff = lat_leader - latitude_follower_2
        longitude_diff = long_leader - longitude_follower_2
        dist_leader = m.sqrt(latitude_diff**2 + longitude_diff**2)#Calculates distance to leader

        theta_error = head_leader - head_follower
        print("Test_2")
        print("Erro theta:",theta_error)

        print("Lider head:",head_leader)
        print("Follower head:",head_follower)

        if(abs(theta_error) >= MINIMUM_ANGLE_FOV):

            theta_derivative_value = theta_error_old - theta_error
            theta_error_old = theta_error

            theta_pid_control = kp_theta*theta_error + kd_theta*theta_derivative_value

            if(theta_pid_control >= MAX_STEERING_ANGLE):
                theta_pid_control = MAX_STEERING_ANGLE
                print("1234")
            elif(theta_pid_control < -MAX_STEERING_ANGLE):
                theta_pid_control = -MAX_STEERING_ANGLE
        
        
        else:
            theta_pid_control = 0
            theta_error = 0
            theta_integral = 0
            theta_error_old = 0
        
        return (-theta_pid_control)
    
    else:
        return 0.0

# def lateral_control_new_algorithm():
    
#     global heading_leader
#     global cross_track_distance
#     global delta_final
#     global speed_follower_2
#     global heading_follower_2
#     global heading_correction
#     global path_heading
#     global lateral_control_time_flag

#     k_stanley = 4.0#Proportional constant

#     # y_d = longitude_leader - longitude_follower_2
#     # x_d = platoon_distance

#     #vf_t = speed_follower_2
#     lat_control_time = time.time()
#     lat_delay = lat_control_time - lateral_control_time_flag

#     if(lat_delay >= MIN_TIME_STAMP):
#         lateral_control_time_flag = lat_control_time#Updates the counter
#         heading_correction = heading_follower_2 - path_heading#Calculates heading error between the orientation of car2 and the actual heading of the vehicle

#         delta_final = heading_correction + m.atan2(k_stanley*cross_track_distance,speed_follower_2)#Calculates the steering angle desired to perform the curve
#         print("Test",delta_final)

#     return delta_final

##########################
# LIDAR MEASUREMENTS
##########################
def lidar_meausurements(data):

    global lidar_coordinates_x
    global lidar_coordinates_y
    global measures_lidar
    global total_distance
    global cross_track_distance
    global lidar_real_dist
    global lidar_sum

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
    #print("Total distance meausured by LiDAR\n", total_distance)#Distance in meters
    time.sleep(0.1)
    
    #lidar_real_dist = data.ranges[360]

    #Create array to store values from a wider range to obtain medium value for distance meausured by LiDAR
    lidar_sum = data.ranges[340:380]#Copies array to lidar_sum array
    sum_total = sum(lidar_sum)#Calculates the sum of the array

    lidar_real_dist = sum_total/len(lidar_sum)#Arithmetic average
    #print("Soma lidar",lidar_real_dist)
    
    #return cross_track_distance


def following_leader():

    global leader_position_array
    global latitude_leader
    global longitude_leader
    global speed_leader
    global latitude_follower_2
    global longitude_follower_2
    global heading_follower_2
    global mode
    global TV_position_vector

    lost_distance = 0
    dist = 0
    dist_lost = 0
    dist_lost_min = 999
    pos_lost = 0
    pos = 0
    dist_min = 999

    #leader_position_array = np.append(leader_position_array,[[latitude_leader,longitude_leader,speed_leader]],axis=0)

    for i in range(TV_position_vector.shape[0]):
        following_lat_diff = TV_position_vector[i,0] - latitude_follower_2
        following_long_diff = TV_position_vector[i,0] - longitude_follower_2

        angle_leader_follower = m.atan2(following_long_diff,following_lat_diff)#Calculates the angle between the positions of leader and follower

        angle_diff = heading_follower_2 - angle_leader_follower#Difference between follower and the angle between leader and follower

        if(abs(angle_diff) < MAXIMUM_ANGLE_FOV):
            print(1)
            dist = m.sqrt((TV_position_vector[i,1] - longitude_follower_2)**2 + (TV_position_vector[i,0] - latitude_follower_2)**2)
        else:
            dist_lost = m.sqrt((TV_position_vector[i,1] - longitude_follower_2)**2 + (TV_position_vector[i,0] - latitude_follower_2)**2)

            if dist_lost_min > dist_lost and dist_lost > 0.0:
                pos_lost = i
                dist_lost_min = dist_lost

        
        if dist_min > dist and dist > 0.0:
            pos = i
            dist_min = dist             #else, update dist_min value
            mode = NORMAL_MODE
        elif dist_min < dist:
            mode = NORMAL_MODE
            break
        elif (i == (TV_position_vector.shape[0] - 1)):
            pos = pos_lost
            mode = LOST_POSITION

    latitude_out = TV_position_vector[pos,0]                 #update the value of TV_latitude 
    longitude_out = TV_position_vector[pos,1]                #update the value of TV_longitude 
    heading_out = TV_position_vector[pos,2]
    speed_out = TV_position_vector[pos,3]#Corrected
    TV_position_vector = TV_position_vector[pos:,:]     #clean the vector, excluding the old positions

    return latitude_out, longitude_out, heading_out, speed_out

#####################################################################
#Compare the latitude and longitude from LiDAR and Gazebo odometry
#####################################################################
def compare_meausures():

    global latitude_leader
    global longitude_leader
    global latitude_leader_compare
    global longitude_leader_compare

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

    lidar_csv_write()#Calls function to write values in CSV

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
    global steering_angle
    global TV_position_vector

    #Follower data
    global longitude_follower_2
    global latitude_follower_2
    global heading_follower_2
    global speed_follower_2
    global steering_angle_car2
    global orientation_x_car2
    global orientation_y_car2
    global speed_new_val

    #Timers
    global time_iteration_2
    global initial_time
    global time_flag

    #PID
    global PID_real_dist
    global platoon_distance

    #Stanley Controller
    global path_heading

    time_now = time.time()
    time_lapse = time_now - time_control_flag

    path_heading = m.atan2(orientation_x_car2,orientation_y_car2)
    #ctrl_msg_car2 = drive_param()

    pub = rospy.Publisher('drive_parameters/car2', drive_param, queue_size=10)

    msg_follower = drive_param()

    if(time_lapse >= MIN_TIME_STAMP):

        if (TV_position_vector[0,0]<=0.1 and TV_position_vector[0,1]<=0.1):#cleaning the first position of the vector
            TV_position_vector = np.append(TV_position_vector,[[latitude_leader, longitude_leader, heading_leader, speed_leader]],axis=0)#Numpy vector que armazena posicoes de TV
            TV_position_vector = TV_position_vector[1:,:]#delete the first line with empty values
        else:
            TV_position_vector = np.append(TV_position_vector,[[latitude_leader, longitude_leader, heading_leader, speed_leader]],axis=0)#Numpy vector que armazena posicoes de TV

        platoon_distance = m.sqrt((latitude_leader - latitude_follower_2)**2 + (longitude_leader - longitude_follower_2)**2)
        print("Distance:", platoon_distance)

        platoon_distance_error = platoon_distance - MAX_DISTANCE
        print("Distance error:", platoon_distance_error)

        #steering_angle_car2 = lateral_control(latitude_leader,longitude_leader,heading_leader,heading_follower_2)
        #heading_follower_2 = heading_leader

        if(platoon_distance >= MIN_DISTANCE and speed_leader >= MIN_SPEED):
            PID_real_dist = longitudinal_control(platoon_distance_error)
            #steering_angle_car2 = lateral_control(latitude_leader,longitude_leader,heading_leader,heading_follower_2)
            #lateral_control_new_algorithm()
            #steering_angle_car2 = lateral_control_new_algorithm()
            print("dist:",PID_real_dist)
            print("dist_1:",platoon_distance)

            lat_compare, long_compare, heading_compare, speed_compare = following_leader()

            speed_new_val = speed_compare#+PID_real_dist)
            # print("speed compare:",speed_new_val)#Debug
            msg_follower.velocity = speed_new_val

            steering_new = lateral_control(lat_compare,long_compare,heading_compare,heading_follower_2)
            # steering_new = steering_new + steering_angle_car2
            

            if(speed_new_val < MIN_SPEED):
                speed_new_val = MIN_SPEED
                msg_follower.velocity = speed_new_val
            elif(speed_new_val > MAX_SPEED):
                speed_new_val = MAX_SPEED
                msg_follower.velocity = speed_new_val
        else:
            speed_new_val = 0.0
        
        # with open(strftime('compare_pid.csv','w')) as compare_pid:
        #     compare_pid.write('%f,%f\n' % (platoon_distance,platoon_distance_error))#Writes distance into file
        
    # platoon_distance = m.sqrt(((longitude_leader - longitude_follower_2)**2) + ((latitude_leader - latitude_follower_2)**2))
    # platoon_distance_error = platoon_distance - MAX_DISTANCE
    # print(platoon_distance_error)

    # #if(platoon_distance > MIN_DISTANCE and speed_leader > MIN_SPEED):
            
    # PID_real_dist = longitudinal_control(platoon_distance_error)
    # print(21)

    # steering_angle = direction_control(latitude_leader,longitude_leader,heading_leader,heading_follower_2)
    msg_follower.velocity = speed_new_val
    msg_follower.angle = steering_angle
    pub.publish(msg_follower)

    csv_file_write()
    compare_meausures()

def listener():
    print("F1/10 node started")
    rospy.init_node('f1_10', anonymous=True)
    rospy.Subscriber('/drive_parameters', drive_param, car_parameters_leader)#Subscribes to topic that stores velocity and steering angle
    #rospy.Subscriber('/drive_parameters/car2', drive_param, car_parameters_car2)
    rospy.Subscriber('/scan/car2', LaserScan, lidar_meausurements)
    rospy.Subscriber('/car2/odom', Odometry, car2_info)
    rospy.Subscriber('/car1/odom', Odometry, car1_info)
    rospy.spin()

if __name__ == '__main__':
    listener()