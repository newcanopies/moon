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
    
    image_width = 400
    image_centroid = image_width / 2
    max_turn = 0.4
    scale = 0.001
    turn = 0.0
    tresh_hold = 20                # relative to centroid
    blob_position = 0
    
    if(len(data.blobs)):
        for obj in data.blobs:
            blob_position = obj.x       # blob_position 

            blob_position = blob_position/len(data.blobs)
            rospy.loginfo("blob is at %s"%blob_position)
            distance_centroid = blob_position - image_centroid # relative position

            if abs(distance_centroid) < tresh_hold:  # no turning within centroid tresh_hold 
		            turn = 0
		            return
            turn_raw = -scale * distance_centroid* max_turn  # scale
            turn_magnitude = max(abs(turn_raw), max_turn)   # velocity
            turn = copysign(turn_magnitude, turn_raw)       # direction

def run():
    global blob_position
    rospy.init_node('RedBall_tracker')      # initialize node 
    pub = rospy.Publisher('/mira/commands/velocity', Twist, queue_size=1) # start publisher for messages of type Twist to velocity
    rospy.Subscriber('/blobs', Blobs, callback) #subscrive to sensor 
    
    global turn
    twist = Twist()

    while not rospy.is_shutdown():
        
        if ( turn != 0.0 ):                 # Yaw turn to track                  
            str = "Turning %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
            turn = 0.0  
        else:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
      
        pub.publish(twist)                   # delay    
        blob_position = 0
        rospy.sleep(0.1)

# import blobsblob.py as a library and run as main program 
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
