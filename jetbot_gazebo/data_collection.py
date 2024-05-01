#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import os
import signal
import threading

class DataCollectionNode(object):
    def __init__(self):
        rospy.init_node('data_collection')

        self.image_subscriber = rospy.Subscriber('image_raw', Image, self.image_listener)
        self.keys_subscriber = rospy.Subscriber('keys', String, self.key_listener)

        self.data_path = rospy.get_param('~data_path', None)
        if self.data_path is None:
            raise ValueError("You must specify a data path parameter.")

        if not os.path.exists(self.data_path):
            os.makedirs(self.data_path)

        self.bridge = CvBridge()
        self.collect = False
        self.xy_label = None

        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signal, frame):
        rospy.signal_shutdown('CTRL-C Pressed')
        cv2.destroyAllWindows()
        sys.exit(0)

    def key_listener(self, msg):
        if msg.data.lower() == 'c':
            self.collect = True

    def image_listener(self, msg):
        if not self.collect:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        cv2.imshow("Image Window", cv_image)
        cv2.setMouseCallback("Image Window", self.click_event)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            rospy.signal_shutdown("ESC key pressed.")
            cv2.destroyAllWindows()

        if self.xy_label:
            img_path = os.path.join(self.data_path, "xy_{0:03d}_{1:03d}_{2}.jpg".format(
                self.xy_label[0], self.xy_label[1], datetime.now().strftime('%Y%m%d-%H%M%S-%f')))
            cv2.imwrite(img_path, cv_image)
            rospy.loginfo("Saved image to: {0}".format(img_path))
            self.collect = False
            self.xy_label = None
            cv2.destroyAllWindows()

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.xy_label = (x, y)

if __name__ == '__main__':
    data_collection_node = DataCollectionNode()
    rospy.spin()