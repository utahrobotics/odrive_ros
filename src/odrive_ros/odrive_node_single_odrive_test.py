#!/usr/bin/env python

import sys
import odrive
from odrive.enums import *
import rospy
from motion_control.msg import Mobility

topic = ''  
front_odrv_port = "serial:/dev/ttyACM0" #the serial port for the odrive

front_odrv = None

#default control is velocity and default state is idle
#message to change state to closed_loop should be sent
control_type = CTRL_MODE_VELOCITY_CONTROL;
odrv_state = AXIS_STATE_IDLE;

#tranformation values to map current and velocity to motor counts
gear_ratio = 1 #the gear reduction from motor to wheel
vel_max = rospy.get_param("max_odrive_velocity", 5000) #in encoder counts per sec, divide by 1024 to get max RPS (rotation per sec)
cur_max = rospy.get_param("max_odrive_current", 30) #message for current control will be a value between -1,1 


def callback(message):
    # odrive stack sperate with one odrive for front wheels and one for  back wheels
    # axis0 (motor 0) is designated for the left wheel motor
    # axis1 (motor 1) is the right wheel motor
    if odrv_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
        if control_type == CTRL_MODE_VELOCITY_CONTROL:
            front_odrv.axis0.controller.vel_ramp_target = message.front_left * vel_max * gear_ratio *-1
            front_odrv.axis1.controller.vel_ramp_target = message.front_right * vel_max * gear_ratio

        elif control_type == CTRL_MODE_CURRENT_CONTROL:
            front_odrv.axis0.controller.current_setpoint(int(message.front_left) * cur_max)
            front_odrv.axis1.controller.current_setpoint(int(message.front_right) * cur_max)



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
    rospy.Subscriber("odrive_vel", Mobility, callback)

    #save for possible use later if the statmachine want to be used to idle the motors
    #rospy.Subscriber(StateMachine, Mobility, control_type_callback);

    #setting up the odrives
    odrive_setup()

    rospy.spin()
    return



def set_control_type(control):
    global control_type
    control_type = control
    #set all odrive motors to given control state
    front_odrv.axis0.controller.config.control_mode = control
    front_odrv.axis1.controller.config.control_mode = control
    return

def set_state(requested_state):
    #set all odrvie motors to a given state
    global odrv_state
    odrv_state = requested_state
    front_odrv.axis0.requested_state = requested_state
    front_odrv.axis1.requested_state = requested_state

    return 

def odrive_setup():
    global front_odrv
    # odrive stack sperate with one odrive for front wheels and one for  back wheels
    # axis0 (motor 0) is designated for the left wheel motor
    # axis1 (motor 1) is the right wheel motor
    
    #serial ports conecting odrives to the robot, to be determined
    

    
    #finding odrives
    rospy.loginfo('finding front wheel odrive')
    front_odrv = odrive.find_any()
    rospy.loginfo('front odrive connected on ' + front_odrv_port)

    
    # odrive callibration
    # assuming motor is already precallibrated, can be done by setting odrv0.axis0.motor.config.pre_caliibrated = true
    #can only do a pre calibrated encoder if using encoder with index(Z) value
    if not(front_odrv.axis0.motor.config.pre_calibrated): #front left motor and encoder
        front_odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    elif not(front_odrv.axis0.encoder.is_ready) or not(front_odrv.axis0.encoder.is_ready):
        front_odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

    if not(front_odrv.axis1.motor.config.pre_calibrated): #front right motor and encoder
        front_odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    elif not(front_odrv.axis1.encoder.is_ready):
        front_odrv.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

    while (front_odrv.axis0.current_state != AXIS_STATE_IDLE or front_odrv.axis0.current_state != AXIS_STATE_IDLE):
	break

    rospy.loginfo("entering control state")
    #set all motors into default state and control mode(active)
    set_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
    set_control_type(CTRL_MODE_VELOCITY_CONTROL)

    #enable velocity ramping
    front_odrv.axis0.controller.vel_ramp_enable = True
    front_odrv.axis1.controller.vel_ramp_enable = True
    return

if __name__ == '__main__':
    listener_init()
