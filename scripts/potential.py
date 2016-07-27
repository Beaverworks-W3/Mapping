#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from racecar import racecar
import numpy as np

class potential:
	def __init__(self):
		# create Subscriber and initialize racecar class
		self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
		self.car = racecar()
		self.push = vector(10.0,0)
	def callBack(self,msg):
		resultList = []
		for i in range(180,900):				# creates array with 960 elements
			average = 0
			for j in range(-2,3):
				average = average+msg.ranges[i+j]	# averages 5 values together
			resultList.append(average/5)
		result = self.push
		for i in range(0,720):
			r = resultList[i]
			vec = vector(-.1/(r*r),(i/4-90))	
			result = result.vectorAdding(vec)
		#print(result.magnitude)
		self.car.drive(0.05*result.magnitude,-2*result.direction)
class vector:
	def __init__(self,magnitude,direction):
		self.magnitude = magnitude
		self.direction = math.radians(direction)
	def vectorAdding(self,vectorTwo):
		magX = self.magnitude * math.cos(self.direction)+vectorTwo.magnitude*math.cos(vectorTwo.direction)
		magY = self.magnitude * math.sin(self.direction)+vectorTwo.magnitude*math.sin(vectorTwo.direction)
		direction = math.atan2(magY,magX) * np.sign(magX)
		magnitude = math.sqrt(magY*magY+magX*magX)*np.sign(magX)
		return vector(magnitude, math.degrees(direction))
if __name__ == "__main__":
	rospy.init_node("potential")
	potential = potential()
	rospy.spin()
