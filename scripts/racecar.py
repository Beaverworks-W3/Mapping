#!/usr/bin/env python
"""
racecar(local).py

MIT RACECAR 2016

This class will import all algorithm modules and
should be imported by the main python script that
activates the node.

"""


# IMPORTS

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



# CLASS DECLARATION

class racecar:

    def __init__(self):
        self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
        self.SafetyPub = rospy.Publisher('vesc/ackermann_cmd_mux/input/safety', AckermannDriveStamped,queue_size=10)        

        self.errorDif = 0
        
        # Add any other topic variables here
    

 
    # Function: drive
    # Parameters: speed (float), angle (float)
    #
    # This function will publish a drive command to
    # the /navigation topic in ROS

    def drive(self, speed, angle):
        msg = AckermannDriveStamped()           # Initializes msg variable
        msg.drive.speed = speed                 # Sets msg speed to entered speed
        msg.drive.acceleration = 0              # Sets msg acceleration to 0
        msg.drive.jerk = 1                      # Sets msg jerk to 1
        msg.drive.steering_angle = angle        # Sets msg steering angle to entered angle
        msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
        self.DrivePub.publish(msg)              # Publishes the message


    
    # Function: calcDistance
    # Parameters: ranges (list), side (string, L or R)
    #
    # This function will estimate a distance from
    # a target 

    def calcDistance(self, ranges, side):
        
        lengthx = 0
        lengthy = 0
        total = 0

        if side == "F":
	    lengthx = ranges[480]
	    lengthy = ranges[600]

	    '''
            total = 0
            for count in range(530, 550):
                total += ranges[count]
            return total/20
	    '''
        
        elif side == "L":                 # Wall on left side
            lengthx = ranges[780]
            lengthy = ranges[900]

        else:                           # Wall on right side
            lengthx = ranges[300]
            lengthy = ranges[180]

        zSquare = lengthx * lengthx + lengthy * lengthy - 2 * lengthx * lengthy * math.sqrt(3)/2
	lengthz = math.sqrt(zSquare)
	
	return ((lengthx * lengthy * 0.5) / lengthz)


    # Function: safety
    # Parameters: ranges (list)
    #
    # This function checks the area in front of
    # it and publishes a stop command if it's
    # about to hit something.

    def safety(self, ranges):

        distance = self.calcDistance(ranges, "F")  # Scans forward

        if distance < 0.6:      # If object < 0.6m ahead
            msg = AckermannDriveStamped()           # Initializes msg variable
            msg.drive.speed = 0                     # Sets msg speed to entered speed
            msg.drive.acceleration = 0              # Sets msg acceleration to 0
            msg.drive.jerk = 1                      # Sets msg jerk to 1
            msg.drive.steering_angle = 0            # Sets msg steering angle to entered angle
            msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
            self.SafetyPub.publish(msg)


    # Function: bbWallFollow
    # Parameters: ranges (list), d_desired (float), speed (float), side(string)
    #
    # This function uses a bang-bang control system
    # to let the car follow a line

    def bbWallFollow(self, ranges, d_desired, speed, side):

        distance = self.calcDistance(ranges, side)   # Queries for distance estimate
        error = d_desired - distance    # Calculates error
        steering_angle = 0              # Initializes steering_angle

        THRESHOLD = 0.05                # Sets threshold to 5cm

        if side == "R":
            if abs(error) < THRESHOLD:      # If error within threshold:
                steering_angle = 0          #       Kill steering

            elif error > 0:                 # If too far to the right:
                steering_angle = 0.5          #       Turn left

            elif error < 0:                 # If too far to the left:
                steering_angle = -0.5         #       Turn right

        else:
            if abs(error) < THRESHOLD:      # If error within threshold:
                steering_angle = 0          #       Kill steering

            elif error > 0:                 # If too far to the right:
                steering_angle = -0.5          #       Turn left

            elif error < 0:                 # If too far to the left:
                steering_angle = 0.5         #       Turn right
        
        self.drive(speed, steering_angle)    # Execute drive function



    # Function: PWallFollow
    # Parameters: ranges (list), d_desired (float), speed (float),
    #             side (string, L or R)
    #
    # This function uses a proportional control system
    # to let the car follow a line

    def PWallFollow(self, ranges, d_desired, speed, side):

        distance = self.calcDistance(ranges, side)  # Finds the minimum range
        error = d_desired - distance    # Calculates error

        steering_angle = 0              # Initializes steering_angle

        THRESHOLD = 0.05                # Sets threshold to 5cm
        Kp = 0.1

        if side == "L":
            steering_angle = -Kp * error

        else:
            steering_angle = Kp * error

        self.drive(speed, steering_angle)    # Execute drive function


    # Function: PDWallFollow
    # Parameters: ranges (list), d_desired (float), speed (float),
    #             side (string, L or R)
    #
    # This function uses a proportional control system
    # to let the car follow a line

    def PDWallFollow(self, ranges, d_desired, speed, side):

        distance = self.calcDistance(ranges, side)  # Finds the minimum range
        error = d_desired - distance    # Calculates error

        steering_angle = 0              # Initializes steering_angle

        THRESHOLD = 0.05                # Sets threshold to 5cm
        Kp = 0.1
        Kd = - 0.2

        steering_angle = (Kp * error) + (Kd * self.errorDif)
        self.errorDif = error - self.errorDif

        self.drive(speed, steering_angle)    # Execute drive function

