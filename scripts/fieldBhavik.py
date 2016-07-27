#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from racecar import racecar
import time
import numpy as np

class getAround:
	def __init__(self):

		# create Subscriber and initialize racecar class
		self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
		self.car = racecar()

	def callBack(self,msg):

		x_net = 0
		y_net = 0

		for i in range(0,1080):				
			distance = msg.ranges[i]
			angle = (i - 540)/4

			magnitude = -.1/(distance * distance)
			
			x_net += np.cos(angle) * magnitude
			y_net += np.sin(angle) * magnitude
				
		
		x_net += 4.0

		steering_angle = 1.0 * math.atan2(y_net,x_net) * np.sign(x_net)
		speed = .5 * math.sqrt(x_net*x_net + y_net*y_net) * np.sign(x_net)

		self.car.drive(speed, steering_angle)

if __name__ == "__main__":
	rospy.init_node("getAround")
	getAround = getAround()
	rospy.spin()
