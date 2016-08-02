#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image as img
from std_msgs.msg import String
from cv_bridge import CvBridge
from fieldYifan import getAround
from racecar import racecar
from color_pickup import colorPicker, contours

#state = 1 => driving towards the blob state = 2 => making the turn
global state = 1
#seen = 1 => haven't finished the turn yet. 2=> finished the turn
global seen = 1
#default color is red
global color = "red"
class color_turn:
    def __init__(self):
        print("inited")
        self.bridge = CvBridge()
        self.state = 1
        self.seen = 1
        self.color = "red"
        self.notTurned = True
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", img, self.camCallback)
        self.colorDic = {
        "red":[0,165,100,6,255,255],
        "green":[60,100,100,70,255,255],
        }
        self.car = racecar()
        self.field = None
        self.picker = colorPicker("no")

    def camCallback(self,msg):
        print("processing image")
        self.contourList = []
        img_data = self.bridge.imgmsg_to_cv2(msg)
        for keys in self.colorDic:
            self.contourCreation(keys,img_data)
        if len(self.contourList)>0:
	        print("got to picker")
            biggest = self.findBiggest(self.contourList)
        if biggest != None:
	        print("found")
            x,y,w,h = cv2.boundingRect(biggest.contour)
            if w*h > 20000: #start turning
                self.state = 2
                self.color = biggest.text
            else:
                error = x+w/2-640
                steering = -0.01*error
                self.car.drive(0.5,steering)
        else:
    	    print("doesn't see anything")
    	    if self.state == 2:
                	self.seen = 2
        self.stateDetect()

    def stateDetect(self):
        if self.state == 2 and self.field == None :
            self.field = getAround()
            if self.color == "red":
                if self.seen == 1:
                    self.field.y_boost = 10
            else:
                if self.seen == 1:
                    self.field.y_boost = -10
        if self.field != None:
            if self.seen == 2 and self.notTurned == True:
                self.notTurned = False
                self.field.y_boost = -self.field.y_boost

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
		if cv2.contourArea(result.contour)>2500:
			return result
		else:
			return None

if __name__ == "__main__":
    rospy.init_node("color_turn")
    colorturn = color_turn()
    rospy.spin()
