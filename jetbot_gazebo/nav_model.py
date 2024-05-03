#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import math
import rospy
import numpy as np
import cv2
from PIL import Image

from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import Twist

from dnn.navigation_model import NavigationModel

class NavModelNode(object):
    """
    Navigation model ROS node that uses PyTorch on camera images
    """
    def __init__(self):
        rospy.init_node('nav_model', anonymous=True)

        # create topics
        self.image_subscriber = rospy.Subscriber('image_raw', ImageMsg, self.image_listener, queue_size=10)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # get node parameters
        self.model_path = rospy.get_param('~model', None)
        self.model_type = rospy.get_param('~type', 'regression')
        self.speed_gain = rospy.get_param('~speed_gain', 0.15)
        self.steering_gain = rospy.get_param('~steering_gain', 0.4)
        self.visualize = rospy.get_param('~visualize', False)
        
        rospy.loginfo("model = {}".format(self.model_path))
        rospy.loginfo("type = {}".format(self.model_type))
        rospy.loginfo("speed_gain = {}".format(self.speed_gain))
        rospy.loginfo("steering_gain = {}".format(self.steering_gain))
        
        if self.model_path is None:
            raise ValueError('must specify PyTorch model path parameter (e.g. model:=/path/to/your/model.pth)')
        
        # load model
        self.model = NavigationModel(self.model_path, type=self.model_type)

    def image_listener(self, msg):
        rospy.logdebug("recieved image: {}x{}, {}".format(msg.width, msg.height, msg.encoding))
        #rospy.logdebug(str(msg.header))

        if msg.encoding not in ['rgb8', 'bgr8']:
            raise ValueError("image encoding is '{}' (expected rgb8 or bgr8)".format(msg.encoding))
            
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        if msg.encoding == 'bgr8':
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
        # predict the center point
        xy = self.model.infer(Image.fromarray(img))
        
        x = xy[0]
        y = (0.5 - xy[1]) / 2.0
        
        # convert to steering angle
        steering_angle = np.arctan2(x, y)
        rospy.loginfo('x={:.2f}  y={:.2f}  angle={:.1f}'.format(x, y, math.degrees(steering_angle)))
                
        # publish velocity message
        twist = Twist()
        
        twist.linear.x = self.speed_gain
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -steering_angle * self.steering_gain
        
        self.velocity_publisher.publish(twist)

        # visualization
        if self.visualize:
            px = min(max(((xy[0] + 1) / 2) * img.shape[1], 0), img.shape[1])
            py = min(max(((xy[1] + 1) / 2) * img.shape[0], 0), img.shape[0])
            
            cv_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv_img = cv2.circle(cv_img, (int(px), int(py)), radius=5, color=(50, 155, 255), thickness=2)
            cv2.imshow("{}/{} Inference".format(rospy.get_namespace(), rospy.get_name()), cv_img)
            cv2.waitKey(1)            
                
    def destroy_node(self):
        twist = Twist()
        
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        rospy.loginfo('shutting down, stopping robot...')
        self.velocity_publisher.publish(twist)
        

def main():
    node = NavModelNode()
    rospy.loginfo("starting processing loop...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()