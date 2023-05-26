#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import time

# Define HOMOGENOUS TRANSFORMATION MATRIX Trc with the form of [[Rrc, p], [0, 0, 0, 1]]
# Where Rrc is a rotation matrix from the robot frame to the camera frame, and p is the position vector of the camera wrt the robot

# This TRANSFORMATION MATRIX was defined by designing and performing a calibration procedure with the camera and robot placed on its respective locations
Trc = np.array([[-0.0348133720650385, 0.945421200545344, 0.0340695605926777, 446.987670932381], 
                [1.01926694684017, 0.0939270806258327, 0.0499145342537423, -86.8691365939225], 
                [-0.0153153064624312, 0.0482478159031277, -1.00257039980047, 1065.23331105263], 
                [(4.33680868994202*(10**(-19))), 0, (-8.67361737988404*(10**(-19))), 1]])

#Define the position vector of the object in the camera frame, this will be overwritten by the data from the camera
pc = np.array([[100], [300], [1000], [1]])

#Define the position vector of the object in the robot frame, this will be the product of the matrix multiplication
pr = np.zeros((4, 1))

#Misc global variables
wait_time = 25     #200 is about 6sec, 100 about 3sec
prev_points = np.zeros((wait_time,3))
prev_published_point = np.zeros((1,3))
i = 0
precision = 30           #in mm

# Define safety variable, used to communicate if its safe for the robot to move
safe_to_move = Point32()
safe_to_move.x = 1

def callback(data):
    point = Point32()
    point = data

    #Insert code for transformation
    global Trc, pc, pr, prev_points, i, prev_published_point, safe_to_move
    
    #Determine if the object position is stable by gathering a series of data points
    prev_points[i,0] = point.x
    prev_points[i,1] = point.y
    prev_points[i,2] = point.z
    
    if i == wait_time-1:
        # Transfer the gathered data points to the respective variables
        x_vals = prev_points[:,0]
        y_vals = prev_points[:,1]
        z_vals = prev_points[:,2]

        # Determine the variance in received coordinates for three axis
        x_range = np.max(x_vals)-np.min(x_vals)
        y_range = np.max(y_vals)-np.min(y_vals)
        z_range = np.max(z_vals)-np.min(z_vals)

        print("Entered wait time")

        diff_w_prev = np.max(np.abs(prev_points[i]-prev_published_point[0]))     #difference between current position and the last one that was published
                                                                                 #this stops the program from publishing the same pos more than once if the object doesnt move
        
        # Run the transformation matrix if variance in three axis has appropriate precision, the safety variable is set, and point has not been published before
        if (x_range<precision) and (y_range<precision) and (z_range<precision) and (diff_w_prev>precision) and (point.z != 0) and (safe_to_move.x == 1):
            
            
            # Define position vector in camera frame
            pc[0,0] = point.x
            pc[1,0] = point.y
            pc[2,0] = point.z

            prev_published_point[0,0] = pc[0,0]   #this is to make sure it doesn't publish the same point more than once if the object doesnt move
            prev_published_point[0,1] = pc[1,0]
            prev_published_point[0,2] = pc[2,0]

            # PERFORM MATRIX MULTIPLICATION to determine the position vector of the object in the robot frame
            pr = np.matmul(Trc, pc)

            # Add calculated position vector data to ROS Point32 variable and publish
            point.x = pr[0,0]
            point.y = pr[1,0]
            point.z = pr[2,0]+150

            publisher.publish(point)
            safe_to_move.x = 0

    #Advance i
    i = i + 1
    if i >= wait_time:
        i = 0
    
#Subscriber for safety topic, dont send commands while robot is moving
def safety_callback(data):

    global safe_to_move
    print("RECEIVED SAFETY SINAL")
    safe_to_move = data


def transformation_calc():
    # Initialize the node
    rospy.init_node('transformation_calc', anonymous=True)
    rate = rospy.Rate(0.5)
    # Publisher
    global publisher
    publisher = rospy.Publisher('/point_in_robot_frame', Point32, queue_size=0)
    # Subscribe to the image topic
    rospy.Subscriber('/point_in_camera_frame', Point32, callback)

    #Subscriber for safety topic, dont send commands while robot is moving
    rospy.Subscriber('/safe_to_move', Point32, safety_callback)

    # Spin until node is shut down
    
    rospy.spin()
    

if __name__ == '__main__':
    
    transformation_calc()