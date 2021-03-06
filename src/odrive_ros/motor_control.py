#!/usr/bin/env python

import sys
import odrive
from odrive.enums import *
import rospy
from std_msgs.py import String
from odrive_interface import ODriveInterfaceSerial
import odrive_node

topic = ''  
front_odrv = None
back_odrv = None

#serial ports conecting odrives to the robot, to be determined
front_odrv_port = 'serial:/dev/ttyOdrvFront' 
back_odrv_port = 'serial:/dev/ttyOdrvBack' 

#default control is velocity and default state is idle
#message to change state to closed_loop should be sent
control_type = CTRL_MODE_VELOCITY_CONTROL;
odrv_state = AXIS_STATE_IDLE;



class robot_drive_motors:

    def __init__(self):
        #initialize parameters
        #tranformation values to map current and velocity to motor counts
        gear_ratio = 14 #the gear reduction from motor to wheel
        vel_max = rospy.get_param("max_odrive_velocity") #in encoder counts per sec, divide by 1024 to get max RPS (rotation per sec)
        cur_max = rospy.get_param("max_odrive_current") #message for current control will be a value between -1,1 

        rospy.Subscriber("motion control", message_type, callback)
        rospy.Subscriber(StateMachine, message_type, control_type_callback);

        #setting up the odrives
        odrive_setup()
        return

    def callback(message):
        # odrive stack sperate with one odrive for front wheels and one for  back wheels
        # axis0 (motor 0) is designated for the left wheel motor
        # axis1 (motor 1) is the right wheel motor
        if odrv_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            if control_type == CTRL_MODE_VELOCITY_CONTROL:
                front_odrv.axis0.controller.vel_setpoint(message.front_left * vel_max * gear_ratio)
                front_odrv.axis1.controller.vel_setpoint(message.front_right * vel_max * gear_ratio)
                back_odrv.axis0.controller.vel_setpoint(message.rear_left * vel_max * gear_ratio)
                back_odrv.axis1.controller.vel_setpoint(message.rear_right * vel_max * gear_ratio)
            elif control_type == CTRL_MODE_CURRENT_CONTROL:
                front_odrv.axis0.controller.current_setpoint(message.front_left * cur_max)
                front_odrv.axis1.controller.current_setpoint(message.front_right * cur_max)
                back_odrv.axis0.controller.current_setpoint(message.rear_left * cur_max)
                back_odrv.axis1.controller.current_setpoint(message.rear_right * cur_max)


        return

    def control_type_callback(message):
        #set control type and configure each odrive to that control
        if message.control_type != control_type:
            control_type = message.control_type
            set_control_type(control_type)

        if message.requested_state != odrv_state:
            odrv_state = message.requested_state
            set_state(odrv_state)
        return


    def listener_init():
        #setting up a ros listener
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("motion control", message_type, callback)
        rospy.Subscriber(StateMachine, message_type, control_type_callback);

        #setting up the odrives
        odrive_setup()

        rospy.spin()
        return



    def set_control_type(control_type):
        #set all odrive motors to given control state
        front_odrv.axis0.controller.config.control_mode = control_type
        front_odrv.axis1.controller.config.control_mode = control_type
        back_odrv.axis0.controller.config.control_mode = control_type
        back_odrv.axis1.controller.config.control_mode = control_type
        return

    def set_state(requested_state):
        #set all odrvie motors to a given state
        front_odrv.axis0.requested_state = requested_state
        front_odrv.axis1.requested_state = requested_state
        back_odrv.axis0.requested_state = requested_state
        back_odrv.axis1.requested_state = requested_state
        return 

    def odrive_setup():
        # odrive stack sperate with one odrive for front wheels and one for  back wheels
        # axis0 (motor 0) is designated for the left wheel motor
        # axis1 (motor 1) is the right wheel motor
    
        #serial ports conecting odrives to the robot, to be determined
        front_odrv_port = 'serial' 

        back_odrv_port = 'serial' 
    
        #finding odrives
        rospy.loginfo('finding front wheel odrive')
        front_odrv = odrive.find_any(front_odrv_port)
        rospy.loginfo('front odrive connected on ' + front_odrv_port)
        back_ordv = odrive.find_any(back_odrv_port)
        rospy.loginfo('back odrive connected on ' + back_odrv_port)
    
        # odrive callibration
        # assuming motor is already precallibrated, can be done by setting odrv0.axis0.motor.config.pre_caliibrated = true
        #can only do a pre calibrated encoder if using encoder with index(Z) value
        if not(front_odrv.axis0.motor.config.pre_calibrated): #front left motor and encoder
            front_odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        elif not(front_odrv.axis0.encoder.is_ready) or not(front_odrv.axis0.encoder.pre_calibrated):
            front_odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        if not(front_odrv.axis1.motor.config.pre_calibrated): #front right motor and encoder
            front_odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        elif not(front_odrv.axis1.encoder.is_ready) or not(front_odrv.axis1.encoder.pre_calibrated):
            front_odrv.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        if not(back_odrv.axis0.motor.config.pre_calibrated): #back left motor and encoder
            back_odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        elif not(back_odrv.axis0.encoder.is_ready) or not(back_odrv.axis0.encoder.pre_calibrated):
            back_odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        return
        if not(back_odrv.axis0.motor.config.pre_calibrated): #back right motor and encoder
            back_odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        elif not(back_odrv.axis1.encoder.is_ready) or not(back_odrv.axis1.encoder.pre_calibrated):
            back_odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        #set all motors into default state and control mode(active)
        set_state(AXIS_STATE_IDLE)
        set_control_type(CTRL_MODE_VELOCITY_CONTROL)
        return

def start_odrives():
    #setting up a ros listener
    rospy.init_node('drive_motors')
    odrive_node = robot_drive_motors();

    rospy.spin()
    return

if __name__ == '__main__':
    listener_init()
