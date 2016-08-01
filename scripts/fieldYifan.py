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
    def __init__(self,y):
		# create Subscriber and initialize racecar class
        self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
        self.pub_goal = rospy.Publisher("~potentialFieldGoal", PointStamped, queue_size=1)
        #self.tfResult = rospy.Subscriber("/tf",TFMessage,self.adjust)

        # create racecar object
        self.car = racecar()

        # VARIABLES
        self.STEERING_CONSTANT = 1.0
        self.SPEED_CONSTANT = 0.06
        self.x_boost = 20
        self.y_boost = y
        self.boost_constant = 0.02


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
        total_x = total_x * self.boost_constant + self.x_boost
        total_y = total_y * self.boost_constant + self.y_boost

        # Transform this gradient vector into a PoseStamped object
        # for visualizing in RVIZ
        visualizer_msg = PointStamped()
        visualizer_msg.header.frame_id = 'base_link'
        visualizer_msg.point.x = total_x
        visualizer_msg.point.y = total_y

        # Publish this goal so that we can see it in RVIZ
        self.pub_goal.publish(visualizer_msg)

        # calculate steering_angle & speed
        steering_angle = ((1-2*self.SPEED_CONSTANT) * np.sign(total_x) * math.atan2(total_y, total_x))
        speed = (self.SPEED_CONSTANT * np.sign(total_x) * math.sqrt(total_x**2 + total_y**2))
        #if steering_angle < 0.05:
        #    steering_angle = 10*steering_angle

        # FINALLY!! MOVE THE CAR
        self.car.drive(speed,steering_angle)


    '''
    Adjusts the y_boost value to ensure that the car explores the entire space.
    '''
    #def adjust(self,msg):
     #   transformation = msg.transforms
      #  movingComp = transformation[0].transform
       # vector = movingComp.translation
        #self.y_boost = -5/(1+5*math.exp(-vector.y))

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("potential_field_node")

    # creates getAround object
    node = getAround(0)

    # keeps node running
    rospy.spin()
