#!/usr/bin/env python


"""
PWallFollowNode(local).py

MIT RACECAR 2016

This program implements the bbWallFollow (bang bang
wall follow) method defined in the racecar.py class.

"""

# IMPORTS

import rospy
import math
from racecar import racecar
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from racecar_12.msg import message_comb
from std_msgs.msg import String

# VARIABLES

DATA_THREAD = "/scan"
PARENT_NODE = "/combined"
CMD_NODE = "/commands"
# CONTROLLER_THREAD = " "
NODE_NAME = 'PDWallFollow'

D_DESIRED = 0.2
SPEED = 1.0

RACER = racecar()
SIDE = None
TURN = "L"
FRONT_DISTANCE = .65


# CALLBACK

def callBack(msg):
	# Query for side
	# <implement here>

	# Query for safety
	#RACER.safety(msg.ranges)

	# Query for P Wall Follow Controller
	distL = msg.ranges[180]
	distR = msg.ranges[900]

	if distL >= distR:
		TURN = "L"
	else:
		TURN = "R"
	if SIDE != None and (msg.ranges[540] < FRONT_DISTANCE):
		print("turning")
		RACER.turn(msg.ranges, TURN)
	elif SIDE != None:
		RACER.PDWallFollow(msg.ranges, D_DESIRE, SPEED, SIDE)
	#else:
		#rospy.loginfo("No direction set! Please tell this node to follow either L or R via {0}".format(CMD_NODE))
		

def state_callback(msg):
	if msg.data == "wallFollowLeft":
		SIDE = "L"
	elif msg.data == "wallFollowRight":
		SIDE = "R"
	else:
		SIDE = None
# MAIN()      

rospy.init_node(NODE_NAME)
scanResult = rospy.Subscriber(DATA_THREAD,LaserScan,callBack)
state  = rospy.Subscriber(CMD_NODE, String, state_callback)

#sideEntry = rospy.Subscriber("", ,)

rospy.spin()
