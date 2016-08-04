#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Vector3, Transform
from tf2_msgs.msg import TFMessage
from racecar_12.msg import message_comb
from racecar import racecar
import time
import numpy as np

class getAround:
    def __init__(self):

		# create Subscriber and initialize racecar class
        self.command = rospy.Subscriber("/commands", String, self.callBack)
        self.sensors = rospy.Subscriber("/scan", LaserScan, self.sensorData)

        #self.tfResult = rospy.Subscriber("/tf",TFMessage,self.adjust)

        # create racecar object
        self.car = racecar()
        self.index = 0
        self.prevDist = 0

        # VARIABLES
        self.STEERING_CONSTANT = 0.3
        self.SPEED_CONSTANT = 0.1
        self.prevAngle = 0
        self.x_boost = 25
        self.y_boost = 0
        self.boost_constant_y = 0.01
        self.boost_constant_x = 0.02
        self.k_d = 0.3
        #self.rate = rospy.Rate(8)
        self.command = "field"

    def sensorData(self, sensor):
        if self.command.find("r") == 0:
            trimmed = float(self.command.lstrip("r"))
            self.x_boost = 25-0.002*trimmed
            print("x_boost = %f"%self.x_boost)
            self.y_boost = -35.0
            self.runField(sensor)

    def callBack(self,msg):
        self.command = msg.data
        

    def runField(self,msg):
        total_x = 0                 # front-back sum of potential fields
        total_y = 0                 # left-right sum of potential fields

        for i in range(0,1080):
            radianAng = msg.angle_min + i* msg.angle_increment
            total_x = total_x - math.cos(radianAng)/(msg.ranges[i]**2)
            total_y = total_y - math.sin(radianAng)/(msg.ranges[i]**2)

        # incorporate constant coefficients

        total_x = total_x * self.boost_constant_x + self.x_boost
        total_y = total_y * self.boost_constant_y + self.y_boost
        #print("x=%f, y=%f"%(total_x,total_y))
  


        print("x=%f, y=%f"%(total_x,total_y))


        # calculate steering_angle & speed
        steering_angle = ((self.STEERING_CONSTANT) * np.sign(total_x) * math.atan2(total_y, total_x))
        deriv = steering_angle - self.prevAngle
        self.prevAngle = steering_angle
        steering_angle = steering_angle + self.k_d * deriv
        speed = (self.SPEED_CONSTANT * np.sign(total_x) * math.sqrt(total_x**2 + total_y**2))

        print("turning")
        # FINALLY!! MOVE THE CAR
        self.car.drive(speed,steering_angle)



if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("potential_field_node")

    # creates getAround object
    node = getAround()

    # keeps node running
    rospy.spin()

