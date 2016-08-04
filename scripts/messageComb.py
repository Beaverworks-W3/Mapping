#!/usr/bin/env python
import rospy
import math
import cv2
from sensor_msgs.msg import LaserScan, Image
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Vector3, Transform
from tf2_msgs.msg import TFMessage
from racecar import racecar
from racecar_12.msg import message_comb
#from weekThree.msg import message_comb
from cv_bridge import CvBridge
import time
import numpy as np


#this class decides which race strats to choose
# three race strategies
# short cut strategy
# straight strategy
# turning strategy
class messageCombBeaver:
    def __init__(self):
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color",
                        Image, self.camCallback)
        self.scanResult = rospy.Subscriber("/scan",LaserScan,self.callBack)
        self.result_pub = rospy.Publisher("/combined",message_comb,
                        queue_size = 1)
        self.bridge = CvBridge()
        self.tfResult = rospy.Subscriber("/tf",TFMessage,self.adjust)
        self.picture = None
        self.ranges = []
        self.intensities = []
        self.angle_min = 0
        self.angle_increment = 0
        self.position = None
        self.rotation = None
        self.rate = rospy.Rate(8)

    def camCallback(self,msg):
        self.picture = msg
        message = message_comb()
        if self.picture != None:
            message.picture = self.picture
        message.angle_min = self.angle_min
        message.angle_increment = self.angle_increment
        if self.position != None:
            message.position = self.position
        if self.rotation != None:
            message.rotation = self.rotation

        message.ranges = self.ranges
        message.intensities = self.intensities
        self.result_pub.publish(message)
        self.rate.sleep()

    def callBack(self,msg):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.ranges = msg.ranges
        self.intensities = msg.intensities
        self.rate.sleep()

    def adjust(self,msg):
        transformation = msg.transforms
        movingComp = transformation[0].transform
        self.position = movingComp.translation
        self.rotation = movingComp.rotation
        self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("message_combiner_beaver")
    combine = messageCombBeaver()
    rospy.spin()

