#!/usr/bin/env python
from sensor_msgs.msg import LaserScan, Image
import rospy

class intensityTest:
    def __init__(self):
        self.sensors = rospy.Subscriber("/scan",LaserScan,self.sensorData)
    def sensorData(self,msg):
        print(msg.intensities[540])
if __name__ == "__main__":
    rospy.init_node("lol")
    test = intensityTest()
    rospy.spin()
