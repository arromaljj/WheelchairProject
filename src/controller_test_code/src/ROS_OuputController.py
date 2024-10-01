#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO

GPIO.setup(GPIO.BCM)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(26,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
GPIO.output(21,1)
GPIO.output(26,1)
GPIO.output(20,1)
GPIO.output(16,1)
GPIO.output(19,0)

class MyController(Controller):

    def __init__(self,**kwargs):
        Controller.__init__(self,**kwargs)
    
    def on_up_arrow_press(self):
        print('Move Forward (Green LED)')
        GPIO.output(21,0)
    
    def on_down_arrow_press(self):
        print('Move Backward (Red LED)')
        GPIO.output(26,0)
    
    def on_right_arrow_press(self):
        print('Rotate Clockwise (Yellow LED)')
        GPIO.output(20,0)
    
    def on_left_arrow_press(self):
        print('Rotate Anti-clockwise')
        GPIO.output(16,0)
    
    def on_up_down_arrow_release(self):
        print('Stop')
        GPIO.output(21,1)
        GPIO.output(20,1)
    
    def on_right_left_arrow_press(self):
        print('Stop')
        GPIO.output(26,1)
        GPIO.output(16,1)
    
    def on_circle_press(self):
        print('Module OFF')
        GPIO.output(19,1)
    
    def on_square_press(self):
        print('Module ON')
        GPIO.output(19,0)

zero_vel = Twist()
forward_vel = Twist()
forward_vel.linear.x = 1
backward_vel = Twist()
backward_vel.linear.x = -1
left_vel = Twist()
left_vel.angular.z = 1
right_vel = Twist()
right_vel.angular.z = -1

controller = MyController(interface="/dev/input/js0",connecting_using_ds4drv=False)
controller.listen()