#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image



class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.move_robot = rospy.Publisher("/cmd_vel", Twist, queue_size= 1)
        self.motionv = Twist()  

    def camera_callback(self,data):
        
        try:
            # store retrieved image in OpenCV variable
            # variable data contains the ROS msg with captured image
            # bgr8 encoding is the OpneCV pre-ROS legacy encoding
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # GUI window displaying contents of the variable "cv_image"
        cv2.imshow("camera_raw", cv_image)
        # waitKey function waits for a keystroke event, delay param is in miliseconds where 0 means forever
        cv2.waitKey(1)

        # CROP to ROI for faster detection
        # conversion bgr2hsv
        cv_new_image = cv_image[240:380,100:400]
        hsv=cv2.cvtColor(cv_new_image, cv2.COLOR_BGR2HSV) 

        # get full image width 
        height, width, channels_orig = cv_new_image.shape

        '''
        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
        >> yellow = np.uint8([[[B,G,R ]]])
        >> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >> print( hsv_yellow )
        [[[ 34 255 255]]
        """
        '''

        # THRESHOLD of HSV colorspace cone for pixel color of interest
        # lower_yellow = np.array([233,235,13]) in RGB space
        lower_yellow = np.array([30,150,150]) # in HSV colorspace, HSV encoding removes variable of color saturation making color recognition faster across changing lighting conditions
        # upper_yellow = np.array([255,255,53])
        upper_yellow = np.array([80,255,255]) # in HSV colorspace

        # mask considers the yellow subset all the different bits in the defined Upper and Lower values
        # MASK filters yellow / not yellow
        # prep for centroid calculation 
        # extracting only ROI features, ie: the yellow line or stars
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # bitwise converts to binary: white for selected color, black blocks-out the rest
        # bitwise binary determines area of the blob to calc centroid
        ROI_mask = cv2.bitwise_and(cv_new_image,cv_new_image, mask= mask) 
    
        # .shape has specific information providing specific values Height Width Channel
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        # Centroids, concentration of color being filtered for
        # CoM extrapolated into image; CoM of image Blobs
        # calculated using integrals, using ImageMoments:

        # obtain coordinates of cropped image where blob tracking centroids occur for the specified Yellow range
        # Calculate centroid of the blob of binary image using ImageMoments
        # detecting the moment of the WHITE part of masked image
        m = cv2.moments(mask, False)
        ImageMoment_is = False
        try:
            # assign cx and cy based on the ImageMoments
            # except: prevents division with 0, instead divide by 2
            # detect the centroid of the line
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            ImageMoment_is = True 
            # avoids python compilation errors
            # outside of a compiled python environment, runs only as runtime error
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            ImageMoment_is = False
        
        rospy.loginfo("Kobuki sees Yellow Line is: " +str(ImageMoment_is))


        # Draw the centroid as a CIRCLE 
        # OpenCV supports drawing variety of things over the images, not only geometric shapes
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(ROI_mask,(int(cx), int(cy)), 10,(0,0,255),-1)

        # centering treshold
   #     tresh_hold = 15

        # image filtering windows
        cv2.imshow("ROI_mask", ROI_mask)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        error_x = cx - width / 2;
        twist_object = Twist();
        twist_object.linear.x = 0.2;
        twist_object.angular.z = -error_x / 100;
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))

        # Make it start turning
        if not ImageMoment_is:
            twist_object.linear.x = 0.0;
            twist_object.angular.z = 0.2;
     #   self.move_robot(twist_object)
        # self.motionv based on width, cx 
        # start kobuki motion vector 
        self.move_robot.publish(twist_object)
        rospy.loginfo("Kobuki glide along Yellow line is: " +str(ImageMoment_is))

    def clean_up(self):
        self.move_robot.clean_class()
        cv2.destroyAllWindows()
    
        
       
        


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
