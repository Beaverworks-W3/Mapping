#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan, Image
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Vector3, Transform
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from racecar import racecar
from weekThree import message_comb
from std_msgs.msg import String
import time
import numpy as np

class racingStrats:
	def __init__(self):
		self.information = rospy.Subscriber("/combined",message_comb,self.race_plan)
		self.strats_pub = rospy.Publisher("/commands",String,queue_size = 1)
		self.colorDic = {
		"red":[0,165,100,6,255,255],
		"green":[40,100,40,88,255,255],
		}
