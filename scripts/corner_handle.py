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
        self.command = rospy.Subscriber("/command", String, self.callBack)
        self.sensors = rospy.Subscriber("/combined", message_comb, self.sensorData)

        #self.tfResult = rospy.Subscriber("/tf",TFMessage,self.adjust)

        # create racecar object
        self.car = racecar()
        self.index = 0
        self.prevDist = 0

        # VARIABLES
        self.STEERING_CONSTANT = 0.5
        self.SPEED_CONSTANT = 0.1
        self.x_boost = 25
        self.y_boost = 0
        self.boost_constant_y = 0.01
        self.boost_constant_x = 0.02
        #self.rate = rospy.Rate(8)

        # initialize sensor Data
        self.ranges = []
        self.angle_min = 0
        self.angle_increment = 0

    def sensorData(self, sensor):
        self.ranges = sensor.ranges
        self.angle_min = sensor.angle_min
        self.angle_increment = sensor.angle_increment

    def callBack(self,msg):
        command = msg.data
        if command.find("g") == 0:
            trimmed = int(command.lstrip("g"))
            self.y_boost = 0.0005*trimmed
            self.runField()
        if command.find("r") == 0:
            trimmed = int(command.lstrip("r"))
            self.x_boost = 25-0.001*trimmed
            self.y_boost = -1.0
            self.runField()

    def runField(self):
        total_x = 0                 # front-back sum of potential fields
        total_y = 0                 # left-right sum of potential fields

        for i in range(0,1080):
            radianAng = self.angle_min + i* self.angle_increment
            total_x = total_x - math.cos(radianAng)/(self.ranges[i]**2)
            total_y = total_y - math.sin(radianAng)/(self.ranges[i]**2)

        # incorporate constant coefficients

        total_x = total_x * self.boost_constant_x + self.x_boost
        total_y = total_y * self.boost_constant_y + self.y_boost
        #print("x=%f, y=%f"%(total_x,total_y))
        if(abs(total_y) < 2):
            total_x = 3*total_x
            total_y = 0


        print("x=%f, y=%f"%(total_x,total_y))


        # calculate steering_angle & speed
        steering_angle = ((self.STEERING_CONSTANT) * np.sign(total_x) * math.atan2(total_y, total_x))
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

