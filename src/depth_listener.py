#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import time

# Define misc global variables
target_time = time.time() + 10.0

x_start = 0
x_end = 0
y_start = 0
y_end = 0
step = 6
thresh = 30 #in mm
table_depth = 900   #in mm
focus_size = 150    #in pixels

index_list = np.array([[0,0]])
center_coord = np.array([])

# Main callback function that runs the object detection algorithm based on the depth data from the camera
# This function gets called everytime a frame of depth data is received and the data gets passed to it
def callback(data):

    # Convert the ROS Image message containing depth data into a numpy array
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    # Get the height and width of the depth image
    height, width = img.shape[:2]

    # Get rough center of the depth image
    x = width // 2
    y = height // 2

    # Gather global variables that are used
    global x_start, x_end, y_start, y_end, focus_size, table_depth, index_list, center_coord

    # Define the area of the image to run the detection on, in this case a square of size ~2*focus_size
    x_start = x - focus_size
    x_end = x + focus_size
    y_start = y - focus_size
    y_end = y + focus_size
    
    # Define and initialize a numpy array to hold the indices for pixels that represent an object
    index_list = np.array([[0,0]])

    # Define and initialize a numpy array to hold the determined coordinates of the center of the object
    # Format [y, x]
    center_coord = np.array([0,0]) #normally initialized to [0,0]

    # Loop through the img[j,i] depth values and get the coordinates for the ones that are above the determined threshold (thresh)
    # In this case the average depth of the table is known from a calibration procedure and we're looking for values smaller than that by a certain threshold

    #START OF COMMETING FOR CALIBRATING TRANS MATRIX: when performing the transformation matrix calibration procedure comment out the following piece of code

    # Loop through relevant image area
    for i in range(x_start,x_end,step):
        for j in range(y_start,y_end,step):
            # Determine if pixel is part of an object
            if img[j,i] < (table_depth-thresh):
                # If pixel is part of an object, capture its indices to the following array
                index_list = np.append(index_list,[[j,i]],0)

    # If an object was detected, find its aproximate center
    if index_list.shape[0]>1:
        index_list = np.delete(index_list,0,0)      #delete the initial value which was used to initialize the variable
        
        # A quick and efficient way to find the center of the object is to find the middle point between the min and max indices that compose the object in both axis
        # This, however, is not perfect when the object is not square with the camera. A better center-finding algorithm can be developed as an improvement to the project
        # This approach works well and runs quickly, which is why it was used in the initial development of the project
        y_max = np.max(index_list[:,0])
        y_min = np.min(index_list[:,0])
        x_max = np.max(index_list[:,1])
        x_min = np.min(index_list[:,1])
        
        # Define aproximate center of the detected object
        center_coord = [(y_max+y_min)//2,(x_max+x_min)//2]

    #END OF COMMETING FOR CALIBRATING TRANS MATRIX

    # Calculate x and y coordinates in mm based on XY calibration
    # The formula below was developed during a XY calibration procedure and it captures the X and Y distance in mm as a function of depth

    #                               milimiters per pixel formula                number of pixels    decimal place
    x_coord = round((((0.0017*img[center_coord[0], center_coord[1]])-0.0161) * (center_coord[1]-x)),1)
    y_coord = round((((0.0017*img[center_coord[0], center_coord[1]])-0.0161) * (center_coord[0]-y)),1)
    
    # Print the determined center coordinates of the object
    print("Position of Object: x:{}mm   y:{}mm   z:{}mm".format(x_coord, y_coord, img[center_coord[0], center_coord[1]]))  

    #CODE FOR LEVELING CAMERA TO THE TABLE: this code aids in leveling the camera to the table by printing the depth measured at the corners. Comment the previous print() and uncomment this one in order to use
    # print("Depth Values for: TL:{}mm   TR:{}mm   BL:{}mm   BR:{}mm".format(img[y_start,x_start], img[y_start,x_end], img[y_end,x_start], img[y_end,x_end]))
    #END CODE FOR LEVELING CAMERA TO TABLE

    # Define a Point32 ROS variable, put data into it, and publish
    point = Point32()
    point.x = x_coord
    point.y = y_coord
    point.z = img[center_coord[0], center_coord[1]]
    publisher.publish(point)
    
# This function helps visualize the object center and the picking space in general by showing the RGB feed of the camera with a marker on the object center
def callback2(data):
    # Convert the ROS Image message to a numpy array
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')


    global center_coord, x_start, x_end, y_start, y_end

    # If an object was detected, put a circular marker on its center coordinates
    if center_coord[0]>0:
        img = cv.circle(img, (center_coord[1],center_coord[0]), 10, (0, 0, 255), -1)

    # A rectangular marker shows the area being analized by the algorithm
    img = cv.rectangle(img, (x_start, y_start), (x_end, y_end), (0, 0, 255), 2)

    # Show RGB camera feed
    cv.imshow("Image", img)
    cv.waitKey(10)


    #CODE FOR CALIBRATING THE TRANSFORMATION MATRIX BETWEEN ROBOT AND CAMERA: when performing the transformation matrix calibration procedure UNcomment the following piece of code
    # global target_time
    # if time.time() >= target_time:
    #     plt.imshow(img)
    #     plt.show()
    #     # Reset target time to 30 seconds in the future
        
    #     target_time = time.time() + 30.0
    #END CODE FOR CALIBRATING THE TRANSFORMATION MATRIX BETWEEN ROBOT AND CAMERA

# Function that initializes nodes, publishers and subscribers upon launch of the program
def depth_listener():
    # Initialize the node
    rospy.init_node('image_subscriber', anonymous=True)
    rate = rospy.Rate(0.5)

    # Initiallize Publisher, this publishes the detected location of the object with respect to the camera frame of reference
    global publisher
    publisher = rospy.Publisher('/point_in_camera_frame', Point32, queue_size=10)

    # Subscribe to the image topics
    # This topic receives the depth data coming from the camera
    rospy.Subscriber('/camera/camera/depth/image_rect_raw', Image, callback)
    # This topic receives regular RGB image data
    rospy.Subscriber('/camera/camera/color/image_raw', Image, callback2)

    
    
    # Spin until node is shut down
    
    rospy.spin()

if __name__ == '__main__':
    
    depth_listener()