#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import numpy as np

class ShoreFollowerDrive:
    def __init__(self):
        self.model_dir_ = rospy.get_param("~model_dir")
        self.linear_vel_ = rospy.get_param('~linear_vel')
        self.twist_factor_ = rospy.get_param('~twist_factor')
        
        self.sess_ = None
        self.load_model()
        
        self.bridge = CvBridge()
        
        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.twist_pub_ = rospy.Publisher("~twist", Twist, queue_size=1)

    def load_model(self):
        # Loads the model
        self.model = tf.keras.models.load_model(self.model_dir_)

    def image_callback(self, data):
        # Image call back and resize to proper shape
        # Regularization is done directly inside the model so we don't have to do it.
        raw = self.bridge.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (0,0), fx = 32.0/data.height, fy=32.0/data.width, interpolation=cv2.INTER_AREA), axis=0).astype(np.float32)
        self.twist_pub_.publish(self.image_to_rot(processed_))

    def image_to_rot(self, img):
        # Reads the image, feed it to the network, get the predictions and act on it.
        out = Twist()
        # Runs the network
        res = self.model(img, training=False)
        # Makes sure that the shape of the network matches the required shape
        assert(res[0].shape[0] == 4)
        #print("%5.2f %5.2f %5.2f" %(res[0],res[1],res[2]))
        #TODO: Use the network output so the robot can drive around the lake
        # returns a geometry_msgs.Twist
        
        res_array=res.numpy()
        #print(res.numpy())
        
        #3 labels: go straight,turn left, turn right, which order ???
        #  go straight
        
        max=res_array[0][0]
        index_max=0
        for i in range(3):
        	if(res_array[0][i+1]>max):
        		max=res_array[0][i+1]
        		index_max=i+1
        if(index_max==0):
        	#print("avance droite")
        	out.linear.x=0.2
        	out.angular.z=-1
        elif(index_max==1):
        	#print("avance gauche")
        	out.linear.x=0.2
        	out.angular.z=1
        elif(index_max==2):
        	#print("avance")
        	out.linear.x=0.2
        elif(index_max==3):
        	#print("arriere")
        	out.linear.x=-0.5
            
            
            
        return out

if __name__ == '__main__':
        rospy.init_node('shore_follower_drive', anonymous=True)
        fp = ShoreFollowerDrive()
        rospy.spin()

