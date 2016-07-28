import rospy as rp
import numpy as np
import cv2
from sensor_msgs.msg import Image, String

#Variable constants
NODE_NAME = "SaveColor"

#
class SaveColor:
	def __init__(self):
		self.bridge = CvBridge()
		
		rp.Subscriber("/camera/rgb/image_rect_color", Image, self.cam_callback)
		
		self.img_pub = rp.Publisher("/exploring_challenge", String, queue_size=10)

	def cam_callback(self,msg):
		rp.loginfo("Image recieved! Processing...")	
		img_data = self.bridge.imgmsg_to_cv2(img)

                processed_img_cv2 = self.process_img(img_data)
                processed_img = self.bridge.cv2_to_imgmsg(processed_img_cv2, "bgr8")

	def process_img(self,img)
		hsv = cv2.cvtColor(img, c2.COLOR_RGB2HSV)

		mask = cv2.inRange(hsv, np.array([50,100,100]), np.array([70, 255, 255]))       
                contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(self.cv_image, contours, -1, (0, 255, 0), 3)


rp.init_node(NODE_NAME)
node = SaveColor()

rp.spin()
