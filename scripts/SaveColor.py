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
                contours_g,hierarchy_g = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contoursRGBYP.append(contours_g)
                #cv2.drawContours(img, contours_g, -1, (0, 255, 0), 3)
		#barley tested
		mask_r = cv2.inRange(hsv, np.array([112,204,178]), np.array([125, 255, 255]))
		contours_r, hierarchy_r = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contoursRGBYP.append(contours_r)
 		#cv2.drawContours(img, contours_r, -1, (120, 0, 0), 3)
		#untested
		mask_b = cv2.inRange(hsv, np.array([0,100,100]), np.array([30, 255, 255]))
		contours_b, hierarchy_b = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		#cv2.drawContours(img, contours_b, -1, (0, 0, 120), 3)

		mask_y = cv2.inRange(hsv, np.array([25,100,100]), np.array([35, 255, 255]))
                contours_y, hierarchy_y = cv2.findContours(mask_y, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                #cv2.drawContours(img, contours_y, -1, (255, 255, 0), 3)

		largestContour = contours_g
		for i in contoursRGBYP:
			if cv2.contourArea(i[0]) > largestContour:
				largestContour = i
		
		cv2.drawContours(img, largestContour, -1, (0, 255, 0), 3)
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
