#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from racecar import racecar

class getAround:
	def __init__(self):
		self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
		self.car = racecar()
	def callBack(self,msg):
		result = values.index(min(values))
		difference = 540-result
		steering = 1.0/difference
		car.drive(0.25,steering)
if __name__ == "__main__":
	rospy.init_node("getAround")
	getAround = getAround()
	rospy.spin()
