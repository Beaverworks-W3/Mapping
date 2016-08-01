#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image as img
from std_msgs.msg import String
from cv_bridge import CvBridge
import PIL
from PIL import ImageFont
from PIL import Image
from PIL import ImageDraw
import random
import time

# calculates histogram values
hrange = [0,180]
srange = [0,256]
ranges = hrange + srange

class colorPicker:
    def __init__(self):

        # bridges ZED camera and cv2
        self.bridge = CvBridge()

        # create ZED subscriber and challenge publisher
        self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", img, self.camCallback)
        self.img_pub = rospy.Publisher("/exploring_challenge", String, queue_size=10)
        self.index = 1
        self.contourList=[]
        self.imgDict = {
        "ari.png":"ari",
        "professor karaman.png":"sertac",
        "cat.png":"cat",
        "racecar.png":"racecar"
        }
        self.colorDic = {
        "red":[0,100,100,15,255,255],
        "blue":[120,150,150,135,255,255],
        "yellow":[23, 150, 150,37,255,255],
        "green":[35,100,100,70,255,255],
        "pink":[340,100,100,360,255,255]
        }

    def camCallback(self,msg):
        self.contourList = []
        img_data = self.bridge.imgmsg_to_cv2(msg)
        for keys in self.colorDic:
            self.contourCreation(keys,img_data)
        if len(contourList)>0:
            biggest = self.findBiggest(self.contourList)
        else:
            biggest = None
        if biggest != None:
            self.actionSave(biggest,img_data)

    def actionSave(self,bigContour,img):
        cv2.drawContours(img, bigContour.contour, -1, (0, 255, 0), 3)
        if bigContour.text != "pink":
            self.saveImg(img,bigContour.text)
            self.img_pub.publish(bigContour.text)
        else:
            x,y,w,h = cv2.boundingRect(biggest.contour)
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            sliced = hsv[x:x+w,y:y+h,:]
            hsvTest = cv2.calcHist(sliced,[0,1],None,[180,256],ranges)
            description = self.checkMatch(hsvTest,self.imgDict)
            self.saveImg(img,description)
            self.img_pub.publish(description)

    def checkMatch(self,hsvUsed,imageDict):
        valList = []
        for keys in imageDict:
            readIn = cv2.imread(keys)
            convert = cv2.cvtColor(readIn, cv2.COLOR_BGR2HSV)
            convert = cv2.calcHist(convert,[0,1],None,[180,256],ranges)
            simVal = cv2.compareHist(hsvUsed,convert,cv2.cv.CV_COMP_CORREL)
            valList.append((simVal,imageDict[keys]))
        result = valList[0]
        for i in range(0,len(valList)):
            if valList[i][0]>result[0]:
                result = valList[i]
        return result[1]

    def saveImg(self,img,text):
		cv2.imwrite("troll.jpeg",img)
		pic = Image.open("troll.jpeg")
		font = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf",25)
		draw = ImageDraw.Draw(pic)
		draw.text((0, 0),text,(255,255,0),font=font)
		rand = self.index
		self.index = self.index + 1
		fileName = "/home/racecar/challenge_photos/"+str(rand)+".jpeg"
		pic.save(fileName,"jpeg")
		self.img_pub.publish(text)

    def contourCreation(self,color,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        s = self.colorDic[color]
        mask = cv2.inRange(hsv, np.array([s[0],s[1],s[2]]), np.array([s[3], s[4], s[5]]))
        contourFound = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.contourAppend(self.contourList,contourFound,color)

    def findBiggest(self,contourList):
		result = contourList[0]
		for x in contourList:
			if cv2.contourArea(x.contour)>cv2.contourArea(result.contour):
				result = x
		if cv2.contourArea(result.contour)>0:
			return result
		else:
			return None

    def contourAppend(self,contourList,contour,color):
		for x in contour[0]:
			appendedStuff = contours(x,color)
			contourList.append(appendedStuff)

class contours:
    def __init__(self,contour,text):
    	self.contour = contour
    	self.text = text

if __name__ == "__main__":
    rospy.init_node("save_color")
    node = saveColor()
    rospy.spin()
