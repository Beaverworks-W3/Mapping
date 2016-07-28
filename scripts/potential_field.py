#!/usr/bin/env python
#import everything
import rospy
import math
import numpy as np
from racecar import racecar
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time

class potentialField:
	def __init__(self):
		self.scanResult=rospy.Subscriber("/scan", LaserScan, self.callback)
		self.car=racecar()


	def callback(self,msg):
		magnitudes = []
		x = []
		y = []
		sumx = 0
		sumy = 0
		pspeed = 1.0
		pangle = 1
		specialX = 15
		for i in range(1080):
			magnitudes.append(-0.1/(msg.ranges[i]**2))
			x.append(magnitudes[i]*math.cos(math.radians(i*0.25-135)))
			y.append(magnitudes[i]*math.sin(math.radians(i*0.25-135)))
			sumx = sumx + x[i]
			sumy = sumy + y[i]
		sumx = sumx + specialX
		speed = pspeed * math.sqrt(sumx**2 + sumy**2) * np.sign(sumx)
		angle = pangle * math.atan2(sumy,sumx) * np.sign(sumx)
		self.car.drive(0.2*speed, angle)

if __name__ == "__main__":
	rospy.init_node("potentialField")
	potential = potentialField()
	rospy.spin()

