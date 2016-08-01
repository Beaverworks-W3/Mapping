#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image as img
from std_msgs.msg import String
from cv_bridge import CvBridge

class color_turn:
    def __init__(self):
        #state = 1 => driving towards the 
        self.state = 1
