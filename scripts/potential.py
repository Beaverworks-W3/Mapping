#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from racecar import racecar

class potential:
	def __init__(self):
		# create Subscriber and initialize racecar class
		self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
		self.car = racecar()
		self.push = vector(2,135)
	def callBack(self,msg):
		result = self.push
		for r in msg.ranges:
			vec = vector(-0.1/(r*r),(msg.ranges.index(r)/4))
			result = result.vectorAdding(vec)
		car.drive(result.magnitude,result.direction)
class vector:
	def __init__(self,magnitude,direction):
		self.magnitude = magnitude
		self.direction = math.radians(direction)
	def vectorAdding(self,vectorTwo):
		magX = self.magnitude * math.cos(self.direction)+vectorTwo.magnitude*math.cos(vectorTwo.direction)
		magY = self.magnitude * math.sin(self.direction)+vectorTwo.magnitude*math.sin(vectorTwo.direction)
		direction = math.atan(magY/magX)
		magnitude = math.sqrt(magY*magY+magX*magX)*magY/abs(magY)
		return vector(magnitude,math.degrees(direction))
if __name__ == "__main__":
	rospy.init_node("potential")
	potential = potential()
	rospy.spin()
