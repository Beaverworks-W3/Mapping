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

		self.car.bothWallFollow(msg.ranges, 1.5)

	#	if min(msg.ranges)<0.2:
	#		self.avoid(msg)
	#	else:
	#		self.follow(msg)
	def avoid(self,msg):
		result = msg.ranges.index(min(msg.ranges))
		difference = 540-result
		steering = 1.0/difference
		self.car.drive(0.25,steering)
	def follow(self,msg):
    		if(msg.ranges[540] < 0.65):
			self.car.turn(msg.ranges, "CCW")
			print("turn")
    		else:
    			self.car.bothWallFollow(msg.ranges, 1.0)

if __name__ == "__main__":
	rospy.init_node("getAround")
	getAround = getAround()
	rospy.spin()
