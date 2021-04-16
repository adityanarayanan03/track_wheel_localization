#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

import numpy as np
import serial
import time
import math

#Establish connection with arduino
data_stream = serial.Serial('/dev/ttyACM3', baudrate = 115200, timeout=0)

#Timing variables
start_time = 0
global_start = 0
first = True

#Empty variables to store the distances and stuff
heading = 0.0
distance_traveled = 0.0
x = 0.0
y = 0.0

#Couple of variables for the delimiter based serial sampling that may or may not be necessary
datum = ""
append = False
data_found = False

def on_callback(data):
    global global_start
    global start_time
    global first
    global heading

    #Establish a true start time
    if first:
        start_time = rospy.get_time()
        global_start = rospy.get_time()
        first = False
    
    global data_stream
    global distance_traveled

    global x
    global y

    global datum
    global append
    global data_found

    #Get the time
    dt = rospy.get_time() - start_time
    start_time = rospy.get_time()

    #Loop to read one full datum, between { and }
    while 1:
        incoming_char = data_stream.read() #Read a single byte

        if append: #We only want to append starting after {
            if incoming_char == '}': #Exit condition
                
                #Obtain the incremental distance, dd
                dist = float(datum)
                dd = distance_traveled - dist
                distance_traveled = dist

                #Reset the datum, reset append variable, and get out of the loop
                datum = ""
                append = False
                break
            else: #Just append if we haven't reached exit condition
                datum += incoming_char

        if (incoming_char=='{'): #sampling start condition simply set append to true
            append = True

    yaw_rate = data.angular_velocity.z + .01 #This is hacky. Calibrate this thing for real later.
    heading += yaw_rate * dt

    #If dd is zero, just don't even bother recomputing.
    if dd != 0:
        x += dd * np.sin(heading)
        y += dd * np.cos(heading)
    
    #Package the data into a readable format and publish on the topic
    p = Pose()

    p.position.x = x; p.position.y = y; p.position.z = 0.0

    p.orientation.x = data.orientation.x
    p.orientation.y = data.orientation.y
    p.orientation.z = data.orientation.z
    p.orientation.w = data.orientation.w

    pub.publish(p)

    #Clear out any residual values.
    #We want only values that come in during the time of reading.
    data_stream.flushInput()

def listener():
    rospy.Subscriber('imu', Imu, on_callback)

    global start_time
    global global_start

    global_start = rospy.get_time()
    start_time = rospy.get_time()
    rospy.spin()

if __name__ == '__main__':
    #Insert a wait because things don't seem to work if proper nodes don't exist at time of calling listener()
    time.sleep(30)
    print('Pose Publisher is Active.')

    #Set up publisher
    pub = rospy.Publisher('pose', Pose, queue_size = 1)
    rospy.init_node('track_wheel_pose', anonymous = True) #Don't actually know what this is/if required.

    listener()