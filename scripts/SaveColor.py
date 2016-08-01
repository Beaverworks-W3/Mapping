#!usr/bin/env python
import rospy as rp
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

#Variable constants
NODE_NAME = "SaveColor"

#
class SaveColor:
	def __init__(self):
		print("initializing")
		self.bridge = CvBridge()
		#prevent the software from making multiple images of same color
		self.shots_created = {"red":False, "green":False, "blue":False, "ari":False, "cat":False, "racecar":False, "professor karaman":False}		

		rp.Subscriber("/camera/rgb/image_rect_color", Image, self.cam_callback)

		self.img_pub = rp.Publisher("/echo/img", Image, queue_size=5)				
		self.challenge_pub = rp.Publisher("/exploring_challenge", String, queue_size=10)

	def cam_callback(self,msg):
		rp.loginfo("Image recieved! Processing...")	
		cv_image = self.bridge.imgmsg_to_cv2(msg)

                processed_img_cv2 = self.process_img(cv_image)
                processed_img = self.bridge.cv2_to_imgmsg(processed_img_cv2, "bgr8")
		self.img_pub.publish(processed_img)

	def process_img(self,img):
		hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

		contoursRGBYP = []

		#tested numbers
		mask_g = cv2.inRange(hsv, np.array([50,100,100]), np.array([70, 255, 255]))

                mask_g = cv2.GaussianBlur(mask_g, (21, 21), 0)
		mask_g = cv2.erode(mask_g, (3,3), iterations=5)
		contours_g,hierarchy_g = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contoursRGBYP.append((contours_g, "green"))
		#barley tested
		mask_r = cv2.inRange(hsv, np.array([100,204,178]), np.array([130, 255, 255]))
		mask_r = cv2.GaussianBlur(mask_r, (21,21), 0)
		mask_r = cv2.GaussianBlur(mask_r, (21,21), 0)
		contours_r, hierarchy_r = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contoursRGBYP.append((contours_r, "red"))
		#untested
		mask_b = cv2.inRange(hsv, np.array([0,100,100]), np.array([30, 255, 255]))
		contours_b, hierarchy_b = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		mask_y = cv2.inRange(hsv, np.array([23,100,160]), np.array([30, 255, 255]))
                contours_y, hierarchy_y = cv2.findContours(mask_y, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contoursRGBYP.append((contours_y, "yellow"))

		largest_contour = None
		largest_area = 150
		current_color = None
		for i in contoursRGBYP:
			#prevent oob exception
			if len(i[0]) == 0: 
				continue

			area = cv2.contourArea(i[0][0])
			if area > largest_area:
				largest_contour = i[0]
				largest_area = area
				current_color = i[1]
		if largest_contour != None:
			cv2.drawContours(img, largest_contour, -1, (0, 255, 0), 3)
			cv2.putText(img, current_color, (0, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (0,0,255), 4)
		if current_color != None and self.shots_created[current_color] == False:
			rp.loginfo("creating image for {0}...".format(current_color))
			cv2.imwrite("{0}.png".format(current_color), img)
			self.shots_created[current_color] = True
			self.challenge_pub.publish(current_color)			
		#add labels?
		'''
		try:
			cnts_g = contours_g[0]

			for c in cnts_g:
				M = cv2.moments(c)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])

				# draw the contour and center of the shape on the image
				cv2.drawContours(img, [c], -1, (0, 255, 0), 3)
				cv2.putText(img, "green", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		except Exception, e:
			print("oops")
		'''
		
		return img

rp.init_node(NODE_NAME)
node = SaveColor()

rp.spin()
