#!/usr/bin/env python
import rospy
import math
import cv2
from sensor_msgs.msg import LaserScan, Image
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Vector3, Transform
from tf2_msgs.msg import TFMessage
from racecar import racecar
from cv_bridge import CvBridge
from racecar_12.msg import message_comb
from color_pickup import contours
from std_msgs.msg import String
import time
import numpy as np


class raceStrats:
    def __init__(self):
        self.subscribe = rospy.Subscriber("/combined",message_comb,self.callBack)
        self.strat_pub = rospy.Publisher("/commands",String,queue_size = 1,latch=True)
        self.bridge = CvBridge()
        self.color = None
        self.freeze = False
        self.pos = 0
        self.savedCommand = "field"
        self.colorDic = {
        "red":[-7,165,100,7,255,255],
        "green":[60,100,40,68,255,255],
        }

    def callBack(self,msg):
        if msg.picture != None:
            self.callBackTwo(msg)
   
    def callBackTwo(self,msg):
        self.contourList = []
        img_data = self.bridge.imgmsg_to_cv2(msg.picture)
        for keys in self.colorDic:
            self.contourCreation(keys,img_data)
        if len(self.contourList)>0:
            biggest = self.findBiggest(self.contourList)
        else:
            biggest = None
        if biggest != None and cv2.contourArea(biggest.contour)>1600: #and self.shapeContour(biggest)=="square":
            command = str(cv2.contourArea(biggest.contour))
            #if biggest.text == "green":
                #command = "g"+command
            #else:
            if biggest.text == "red":
                command = "r"+command
            self.strat_pub.publish(command)
            self.savedCommand = command
            self.freeze = True
            self.pos = msg.position.x+msg.position.y
        else:
            self.secondLoop(msg)

    def secondLoop(self,msg):
        if self.savedCommand.find("r")==0:
            if abs(msg.ranges[180]-msg.ranges[900])<1.8 or msg.ranges[540]<2.0:
                self.strat_pub.publish("field")
                self.savedCommand = "field"
        else:
            self.strat_pub.publish("field")

    def contourAppend(self,contourList,contour,color):
        for x in contour[0]:
            appendedStuff = contours(x,color)
            contourList.append(appendedStuff)

    def contourCreation(self,color,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        s = self.colorDic[color]
        mask = cv2.inRange(hsv, np.array([s[0],s[1],s[2]]), np.array([s[3], s[4], s[5]]))
        mask = cv2.GaussianBlur(mask, (21,21), 0)
        mask = cv2.erode(mask, (3, 3), iterations=5)
        contourFound = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.contourAppend(self.contourList,contourFound,color)

    def findBiggest(self,contourList):
        result = contourList[0]
        for x in contourList:
            if cv2.contourArea(x.contour)>cv2.contourArea(result.contour):
                result = x
        if cv2.contourArea(result.contour)>1000:
            return result
        else:
            return None

    def shapeContour(self, cnt):
        epsilon = 0.021*cv2.arcLength(cnt.contour, True)
        approx = cv2.approxPolyDP(cnt.contour, epsilon, True)
        size = len(approx)
        print(size)
        if(size < 8):
            return "square"
        elif(size < 11):
            return "circle"
        else:
            return "cross"


if __name__ == "__main__":
    rospy.init_node("race_strats")
    strats = raceStrats()
    rospy.spin()

