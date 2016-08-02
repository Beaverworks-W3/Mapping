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
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", img, self.camCallback)
        self.colorDic = {
        "red":[0,165,100,6,255,255],
        "green":[60,100,100,70,255,255],
        }
        self.car = racecar()
        self.picker = colorPicker("no")
    def camCallback(self,msg):
        print("processing image")
        self.picker.contourList = []
        img_data = self.picker.bridge.imgmsg_to_cv2(msg)
        for keys in self.colorDic:
            self.picker.contourCreation(keys,img_data)
        if len(self.picker.contourList)>0:
	    print("got to picker")
            biggest = self.picker.findBiggest(self.picker.contourList)
            print("found")
        if biggest != None:
	    print("found")
            x,y,w,h = cv2.boundingRect(biggest.contour)
            if w*h > 20000: #start turning
                state = 2
                color = biggest.text
            else:
                error = x+w/2-640
                steering = -0.01*error
                self.car.drive(0.5,steering)
        else:
	    print("wtf")
	    if state == 2:
            	seen = 2
if __name__ == "__main__":
    rospy.init_node("color_turn")
    colorturn = color_turn()
    if colorturn.state == 2:
        if colorturn.color == "red":
            if colorturn.seen == 1:
                turn = getAround(10)
            else:
                turn = getAround(-10)
        else:
            if colorturn.seen == 1:
                turn = getAround(-10)
            else:
                turn = getAround(10)
    rospy.spin()
