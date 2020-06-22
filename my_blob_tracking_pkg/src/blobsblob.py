#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs, Blob
from math import *

turn = 0.0                              # global
blob_position = 0 

def callback(data):
    global turn
    global blob_position
    global blob_updown
    
    image_width = 400
    image_centroid = image_width / 2
    max_turn = 0.2
    scale = 0.001
    turn = 0.0
    tresh_hold = 15                # relative to centroid
    blob_position = 0
    
    if(len(data.blobs)):
        for obj in data.blobs:
            blob_position = obj.x       # blob_position within x axis
            blob_position = blob_position/len(data.blobs)
            rospy.loginfo("blob is at %s"%blob_position)
            distance_centroid = blob_position - image_centroid # relative horizontal position 

            blob_updown = obj.y        # blob_updown position along y axis of camera image plane
            blob_updown = blob_updown/len(data.blobs)
            rospy.loginfo("blob elevation is %s"%blob_position)
            distance_elevation = blob_updown - image_centroid # relative vertical position

            if abs(distance_centroid) < tresh_hold:  # horizontal is within centroid tresh_hold, no need to turn
                turn = 0
                return
                turn_raw = -scale * distance_centroid * max_turn  # scale 
                turn_magnitude = max(abs(turn_raw), max_turn)   # velocity
                turn = copysign(turn_magnitude, turn_raw)       # direction

            if abs(distance_elevation) < tresh_hold: # if elvation is within treshold, no need to rutn      
                turn_y = 0                                                                                                                                                                                                      
                return
                
                turn_raw_y = -scale * distance_elevation * max_turn  # scale for up and down
                turn_magnitude_y = max(abs(turn_raw_y), max_turn)   # velocity for up and down
                turn_y = copysign(turn_magnitude, turn_raw_y)       # direction up and down 
         #  pass
           

def run():
    global blob_position
    global blob_updown
    rospy.init_node('RedBall_tracker')      # initialize node 
    pub = rospy.Publisher('/mira/commands/velocity', Twist, queue_size=1) # start publisher for messages of type Twist to velocity
    rospy.Subscriber('/blobs', Blobs, callback) #subscrive to sensor 
    
    global turn
    global turn_y
    twist = Twist()

    while not rospy.is_shutdown():
        
        if ( turn != 0.0 ):                 # Yaw z-axis turn to track or search                 
            str = "Turning %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
            turn = 0.0  
            
        elif ( turn > 0.0 ):
            str = "Turning %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = turn_y; twist.angular.z = 0  # Roll axis turn to track Ball up & down
            turn_y = 0.0  

        else:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
      
        pub.publish(twist)                   # delay    
        blob_position = 0
        blob_updown= 0
        rospy.sleep(0.1)

# import blobsblob.py as a library and run as main program 
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass