#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError


import tf
import tensorflow
import numpy as np

class ShoreFollowerDrive:
    def __init__(self):
        self.model_dir_ = rospy.get_param("~model_dir")
        self.linear_vel_ = rospy.get_param('~linear_vel')
        self.twist_factor_ = rospy.get_param('~twist_factor')
        
        self.sess_ = None
        self.load_model()
        
        self.previous_linear_x=0
        self.alpha_x=0.95
        
        self.previous_linear_z=0
        self.alpha_z=0.95
        
        self.bridge = CvBridge()
        
        self.listener_=tf.TransformListener()
        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.twist_pub_ = rospy.Publisher("~twist", Twist, queue_size=1)

    def load_model(self):
        # Loads the model
        self.model = tensorflow.keras.models.load_model(self.model_dir_)

    def image_callback(self, data):
        # Image call back and resize to proper shape
        # Regularization is done directly inside the model so we don't have to do it.
        raw = self.bridge.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (0,0), fx = 32.0/data.height, fy=32.0/data.width, interpolation=cv2.INTER_AREA), axis=0).astype(np.float32)
        self.twist_pub_.publish(self.image_to_rot(processed_))

    def control_z(self, z):
    	if(0.5>z):
    		return 0.2
    	if(0.5<z):
    		return -0.2

    def image_to_rot(self, img):
        # Reads the image, feed it to the network, get the predictions and act on it.
        out = Twist()
        # Runs the network
        res = self.model(img, training=False)
        # Makes sure that the shape of the network matches the required shape
        assert(res[0].shape[0] == 2)
        #print("%5.2f %5.2f %5.2f" %(res[0],res[1],res[2]))
        #TODO: Use the network output so the robot can drive around the lake
        # returns a geometry_msgs.Twist
        
        res_array=res.numpy()
        #print(res.numpy())
        
        
        self.listener_.waitForTransform("/world","/VSV/Tool",rospy.Time(0),rospy.Duration(1.0))
        transform, _ =self.listener_.lookupTransform("/world","/VSV/Tool",rospy.Time(0))
        
        #linear z command with filter
        linear_z=0
        linear_z=self.control_z(transform[2])
        
        linear_z=self.alpha_z*self.previous_linear_z + (1-self.alpha_z)*linear_z
        
        self.previous_linear_z=linear_z
        
        out.linear.z=linear_z
        
        #linear x command with filter
        linear_x=0
        if (res_array[0][0]>=res_array[0][1]):
           	linear_x= -2
        	#  go back
        elif (res_array[0][1]>=res_array[0][0]):
            	linear_x = 2
        
        linear_x=self.alpha_x*self.previous_linear_x + (1-self.alpha_x)*linear_x

        self.previous_linear_x=linear_x
        
        #prevent the tool to destroy the truck
        if(abs(transform[1])>0.85 or linear_x>=0): #if tool is not too close to the truck or we want to extend the arm
        	out.linear.x=linear_x
        else:
        	out.linear.x=0
        
        return out

if __name__ == '__main__':
        rospy.init_node('shore_follower_drive', anonymous=True)
        fp = ShoreFollowerDrive()
        rospy.spin()

