# Standard imports
import cv2
import numpy as np;
import imutils

# calculates histogram values
hrange = [0,180]
srange = [0,256]
ranges = hrange + srange

imgDict = {
        "ari.png":"ari",
        "professor karaman.png":"sertac",
        "cat.png":"cat",
        "racecar.png":"racecar"
        }

# Read image
im = cv2.imread("IMG_1704.JPG")
cv2.namedWindow("hey", cv2.WINDOW_NORMAL)



hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

r1 = np.array([155, 0, 0])

r2 = np.array([175, 255, 255])

mask = cv2.inRange(hsv, r1, r2)
mask = cv2.GaussianBlur(mask, (21,21), 0)
mask = cv2.erode(mask, (3, 3), iterations=5)


contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

result = contours[0]
for x in contours:
	if cv2.contourArea(x)>cv2.contourArea(result):
		result = x

padding = 30
x,y,w,h = cv2.boundingRect(result)
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

print(result[1])



cv2.imshow("hey", sliced)

#hsvTest = cv2.calcHist(sliced,[0,1],None,[180,256],ranges)
'''
description = self.checkMatch(hsvTest,self.imgDict)
self.saveImg(img,description)
self.img_pub.publish(description)

'''

'''
mask = cv2.GaussianBlur(mask, (21,21), 0)
mask = cv2.erode(mask, (3, 3), iterations=5)
contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(im, contours, -1, (0, 255, 0))

for j in range(0, len(contours)):
    #rect = cv2.minAreaRect(contours[j])
    #box = cv2.cv.BoxPoints(rect)
    #box = np.int0(box)
    #cv2.drawContours(im,[box],0,(0,0,255),2)

    M = cv2.moments(contours[j])
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    # draw the contour and center of the shape on the image
    cv2.drawContours(im, [contours[j]], -1, (0, 255, 0), 2)
    cv2.circle(im, (cX, cY), 4, (255, 255, 255), -1)
    cv2.putText(im, "center", (cX - 20, cY - 20),
    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# Show blobs
cv2.imshow("o shit wadap", im)

'''
cv2.waitKey(0)
