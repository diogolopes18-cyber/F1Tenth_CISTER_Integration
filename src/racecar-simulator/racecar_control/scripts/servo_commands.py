#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

# def set_throttle_steer(data):

#     global flag_move

#     pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
#     pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
#     pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
#     pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

#     pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
#     pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    
#     throttle = data.drive.speed/0.1
#     steer = data.drive.steering_angle

#     pub_vel_left_rear_wheel.publish(throttle)
#     pub_vel_right_rear_wheel.publish(throttle)
#     pub_vel_left_front_wheel.publish(throttle)
#     pub_vel_right_front_wheel.publish(throttle)
#     pub_pos_left_steering_hinge.publish(steer)
#     pub_pos_right_steering_hinge.publish(steer)


#######################
#CAR1
#######################
def set_car1_control(data):
    
    pub_vel_left_rear_wheel_car1 = rospy.Publisher('/car1/left_rear_wheel_velocity_controller_car1/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel_car1 = rospy.Publisher('/car1/right_rear_wheel_velocity_controller_car1/command', Float64, queue_size=1)
    pub_vel_left_front_wheel_car1 = rospy.Publisher('/car1/left_front_wheel_velocity_controller_car1/command', Float64, queue_size=1)
    pub_vel_right_front_wheel_car1 = rospy.Publisher('/car1/right_front_wheel_velocity_controller_car1/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge_car1 = rospy.Publisher('/car1/left_steering_hinge_position_controller_car1/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge_car1 = rospy.Publisher('/car1/right_steering_hinge_position_controller_car1/command', Float64, queue_size=1)

    throttle_car1 = data.drive.speed/0.1
    steer_car1 = data.drive.steering_angle

    pub_vel_left_rear_wheel_car1.publish(throttle_car1)
    pub_vel_right_rear_wheel_car1.publish(throttle_car1)
    pub_vel_left_front_wheel_car1.publish(throttle_car1)
    pub_vel_right_front_wheel_car1.publish(throttle_car1)
    pub_pos_left_steering_hinge_car1.publish(steer_car1)
    pub_pos_right_steering_hinge_car1.publish(steer_car1)



#######################
#CAR2
#######################
def set_car2_control(data):

    pub_vel_left_rear_wheel_car2 = rospy.Publisher('/car2/left_rear_wheel_velocity_controller_car2/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel_car2 = rospy.Publisher('/car2/right_rear_wheel_velocity_controller_car2/command', Float64, queue_size=1)
    pub_vel_left_front_wheel_car2 = rospy.Publisher('/car2/left_front_wheel_velocity_controller_car2/command', Float64, queue_size=1)
    pub_vel_right_front_wheel_car2 = rospy.Publisher('/car2/right_front_wheel_velocity_controller_car2/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge_car2 = rospy.Publisher('/car2/left_steering_hinge_position_controller_car2/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge_car2 = rospy.Publisher('/car2/right_steering_hinge_position_controller_car2/command', Float64, queue_size=1)


    throttle_car2 = data.drive.speed/0.1
    steer_car2 = data.drive.steering_angle

    pub_vel_left_rear_wheel_car2.publish(throttle_car2)
    pub_vel_right_rear_wheel_car2.publish(throttle_car2)
    pub_vel_left_front_wheel_car2.publish(throttle_car2)
    pub_vel_right_front_wheel_car2.publish(throttle_car2)
    pub_pos_left_steering_hinge_car2.publish(steer_car2)
    pub_pos_right_steering_hinge_car2.publish(steer_car2)


def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    #rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
    rospy.Subscriber("/car1/ackermann_cmd_mux/output", AckermannDriveStamped, set_car1_control)
    rospy.Subscriber("/car2/ackermann_cmd_mux/output", AckermannDriveStamped, set_car2_control)
    #rospy.Subscriber("/car3/ackermann_cmd_mux/output", AckermannDriveStamped, set_car3_control)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
