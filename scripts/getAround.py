#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from racecar import racecar
import time

class getAround:
	def __init__(self):
		self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
		self.car = racecar()
	def callBack(self,msg):
		resultList = []
		#resultList.append(msg.ranges[0])
		#resultList.append(msg.ranges[1])
		for i in range(60,1020):
			average = 0
			for j in range(-2,3):
				average = average+msg.ranges[i+j]
			resultList.append(average/5)
		#resultList.append(msg.ranges[1078])
		#resultList.append(msg.ranges[1079])
		print(resultList[480])
		if min(resultList)<0.3:
			self.avoid(resultList)
			print("avoiding")
		else:
			self.follow(resultList)
			
	def avoid(self,msg):
		result = msg.index(min(msg))
		#print(result)
		difference = 480-result if (480-result) != 0 else 1
		steering = 200.0/difference if abs(200.0/difference)<1 else difference/abs(difference)
		#print(steering)
		if msg[480]<0.2:
			self.car.drive(-0.25,0.0)
			print("going back")
			time.sleep(0.1)
		else:
			self.car.drive(0.5,steering)
			
	def follow(self,msg):
    		if(msg[480] < 0.6):
			self.car.turn(msg, "CCW")
			print("turn")
    		else:
    			self.car.drive(1.0,0.0)
			print("wallfollowing")

if __name__ == "__main__":
	rospy.init_node("getAround")
	getAround = getAround()
	rospy.spin()