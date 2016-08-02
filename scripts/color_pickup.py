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
        self.rqt_pub = rospy.Publisher("/video", img, queue_size=10)
        self.index = 1
        self.contourList=[]
        self.imgDict = {
        "ari.png":"ari",
        "professor karaman.png":"sertac",
        "cat.png":"cat",
        "racecar.png":"racecar"
        }
        self.colorDic = {

        "red":[0,165,100,6,255,255],
        "blue":[100,150,150,135,255,255],
        "yellow":[25, 150, 150,30,255,255],
        "green":[40,100,40,88,255,255],
        "pink":[165,100,100,170,255,255]
        }

    def camCallback(self,msg):
        self.contourList = []
        img_data = self.bridge.imgmsg_to_cv2(msg)

        for keys in self.colorDic:
            	self.contourCreation(keys,img_data)
        if len(self.contourList)>0:
            	biggest = self.findBiggest(self.contourList)
    	else:
    		self.rqt_pub.publish(msg)

    	if biggest != None:
    		self.actionSave(biggest,img_data)
    	else:
    		self.rqt_pub.publish(msg)

    def actionSave(self,bigContour,img):
        cv2.drawContours(img, bigContour.contour, -1, (0, 255, 0), 3)
        if bigContour.text != "pink":
    		bigContour.text += " " + self.shapeContour(bigContour)
    		print(bigContour.text)
        	self.saveImg(img,bigContour.text)
		self.img_pub.publish(bigContour.text)
        else:
            x,y,w,h = cv2.boundingRect(bigContour.contour)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	    padding = 30

	    r1 = np.array([155, 0, 0])
	    r2 = np.array([175, 255, 255])

            sliced = hsv[y + padding:y+h - padding, x + padding:x+w - padding, :]

	    mask2 = cv2.inRange(sliced, r1, r2)
	    mask2 = cv2.bitwise_not(mask2)
	    mask2 = cv2.GaussianBlur(mask2, (21,21), 0)
	    mask2 = cv2.erode(mask2, (3, 3), iterations=5)

	    contours2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

	    largest = contours2[0]
	    second_largest = contours2[0]
	    for x in contours2:
		if cv2.contourArea(x)>cv2.contourArea(largest):
			second_largest = largest
			largest = x
		elif cv2.contourArea(x) > cv2.contourArea(second_largest):
			second_largest = x

	    x1,y1,w1,h1 = cv2.boundingRect(second_largest)
	    sliced = sliced[y1:y1+h1, x1:x1+w1, :]

            description = self.checkMatch(sliced, self.imgDict)

            self.saveImg(img,description)
            self.img_pub.publish(description)

    def shapeContour(self, cnt):
	epsilon = 0.1*cv2.arcLength(cnt.contour, True)
	approx = cv2.approxPolyDP(cnt.contour, epsilon, True) 

	
	size = len(approx)
	if(size < 7):
		return "square"
	elif(size < 14):
		return "plus"
	else:
		return "circle"



    def checkMatch(self,sliced, imgDict):
        valList = []

	# Initiate SIFT detector
	orb = cv2.ORB()

	des1 = orb.detectAndCompute(sliced,None)[1]

	for keys in imgDict:
		readIn = cv2.imread(keys)

		convert = cv2.cvtColor(readIn, cv2.COLOR_HSV2BGR)
		des2 = orb.detectAndCompute(convert,None)[1]
	
		# create BFMatcher object
		bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

		# Match descriptors.
		matches = bf.match(des1,des2)
		valList.append((len(matches),imgDict[keys]))

	result = valList[0]
	for i in range(0,len(valList)):
		if valList[i][0]>result[0]:
			result = valList[i]

	return result[1]

    def saveImg(self,img,text):
    	cv2.imwrite("troll.jpeg",img)
    	pic = Image.open("troll.jpeg")
    	font = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-B.ttf",50)
    	draw = ImageDraw.Draw(pic)
    	draw.text((0, 0),text,(255,255,0),font=font)
    	rand = self.index
    	self.index = self.index + 1
    	fileName = "/home/racecar/challenge_photos/"+str(rand)+".jpeg"
    	pic.save(fileName,"jpeg")
        published = cv2.imread(fileName)
        published_msg = self.bridge.cv2_to_imgmsg(published)
        self.rqt_pub.publish(published_msg)
	self.img_pub.publish(text)

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
    node = colorPicker()
    rospy.spin()
