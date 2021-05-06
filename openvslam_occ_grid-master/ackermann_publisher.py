#!/usr/bin/env python
import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

class ackermann_publisher:
    def __init__(self):
        self.sub = rospy.Subscriber('ackermann_steering', Float32, self.convert_steering_publish_ackermann, queue_size=1)
        self.pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.ack_msg = AckermannDriveStamped()
        self.ack_msg.drive.speed = 0.3
        self.steering = self.ack_msg.drive.steering_angle
        self.subcribed_result = True

    def convert_steering_publish_ackermann(self,std_msg):
        if(std_msg.data == 2000): # stop!!!
            self.steering = round(std_msg.data,3)
            self.ack_msg.drive.speed = 0

            print("stop")
            print("speed",self.ack_msg.drive.speed)
            print("steering",self.steering)
        else:
            self.steering = round(std_msg.data,3)
            self.ack_msg.drive.steering_angle = -self.steering
            
            print("Drive")
            print("speed",self.ack_msg.drive.speed)
            print("steering",self.steering)

        #self.ack_msg.header.stamp = rospy.Time.now()
        #self.ack_msg.header.frame_id = "a"
        self.pub.publish(self.ack_msg)


if __name__ == "__main__":
    rospy.init_node("ackermann_publisher", anonymous=True)

    r = rospy.Rate(30) # 10hz

    while not rospy.is_shutdown():
        p = ackermann_publisher()
        # print("Running")
        r.sleep()

