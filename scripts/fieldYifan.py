#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Vector3, Transform
from tf2_msgs.msg import TFMessage
from racecar import racecar
import time
import numpy as np

class getAround:
    def __init__(self):
		# create Subscriber and initialize racecar class
        self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
        self.pub_goal = rospy.Publisher("~potentialFieldGoal", PointStamped, queue_size=1)
        #self.tfResult = rospy.Subscriber("/tf",TFMessage,self.adjust)

        # create racecar object
        self.car = racecar()
	self.index = 0
	self.prevDist = 0

        # VARIABLES
        self.STEERING_CONSTANT = 0.4
        self.SPEED_CONSTANT = 0.1
        self.x_boost = 30
        self.y_boost = 0
	self.drivingSpeed = 0
        self.boost_constant = 0.02
	self.prev_x = 0
	self.boost_constant_y = 0.01
	self.boost_constant_x = 0.025
	self.drivingAngle = 0
	#self.rate = rospy.Rate(8)

    '''
    Callback function for scan subscriber
    '''
    def callBack(self,msg):
	
        total_x = 0                 # front-back sum of potential fields
        total_y = 0                 # left-right sum of potential fields
        for i in range(0,1080):
            radianAng = msg.angle_min + i*msg.angle_increment
            total_x = total_x - math.cos(radianAng)/(msg.ranges[i]**2)
            total_y = total_y - math.sin(radianAng)/(msg.ranges[i]**2)

        # incorporate constant coefficients
        total_x = total_x * self.boost_constant_x + self.x_boost - self.drivingSpeed*7
        total_y = total_y * self.boost_constant_y + self.y_boost + self.drivingAngle * 10
	
	#if abs(total_x)<10:
	#	total_x = 10 *np.sign(total_x)
        # Transform this gradient vector into a PoseStamped object
        # for visualizing in RVIZ
        visualizer_msg = PointStamped()
        visualizer_msg.header.frame_id = 'base_link'
        visualizer_msg.point.x = total_x
        visualizer_msg.point.y = total_y

        # Publish this goal so that we can see it in RVIZ
        self.pub_goal.publish(visualizer_msg)
	print("x=%f,y=%f"%(total_x,total_y))
	#print("y=%f"%total_y)
        # calculate steering_angle & speed
        steering_angle = ((self.STEERING_CONSTANT) * np.sign(total_x) * math.atan2(total_y, total_x))
        speed = (self.SPEED_CONSTANT * np.sign(total_x) * math.sqrt(total_x**2 + total_y**2))
	self.drivingSpeed = speed
	self.drivingAngle = steering_angle
        #if steering_angle < 0.05:
        #    steering_angle = 10*steering_angle

        # FINALLY!! MOVE THE CAR
        self.car.drive(speed,steering_angle)
	#self.rate.sleep()

    '''
    Adjusts the y_boost value to ensure that the car explores the entire space.
    '''
    def adjust(self,msg):
	self.index = self.index+1
        transformation = msg.transforms
        movingComp = transformation[0].transform
        vector = movingComp.translation
	
        self.y_boost = 5/(1+5*math.exp(-vector.y))
	#self.rate.sleep()

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("potential_field_node")

    # creates getAround object
    node = getAround()

    # keeps node running
    rospy.spin()
