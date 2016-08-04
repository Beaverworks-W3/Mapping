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
    
        print("x=%f,y=%f"%(total_x,total_y))

        if(total_y < 2):
            total_x *= 3

        steering_angle = ((self.STEERING_CONSTANT) * np.sign(total_x) * math.atan2(total_y, total_x))
        speed = (self.SPEED_CONSTANT * np.sign(total_x) * math.sqrt(total_x**2 + total_y**2))
        self.drivingSpeed = speed
        self.drivingAngle = steering_angle
        
        self.car.drive(speed,steering_angle)

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("potential_field_node")

    # creates getAround object
    node = getAround()

    # keeps node running
    rospy.spin()
