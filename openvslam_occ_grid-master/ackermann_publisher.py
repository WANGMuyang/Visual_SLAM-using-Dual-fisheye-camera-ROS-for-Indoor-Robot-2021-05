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
        self.ack_msg.drive.speed = 0.4
        self.steering = float(0.00)
        self.subcribed_result = True

    def convert_steering_publish_ackermann(self,std_msg):
        self.steering = round(std_msg.data,2)

        print("steering_angle",self.steering)

        if(self.steering == 100): # stop!!!
            self.ack_msg.drive.speed = 0.0
            self.ack_msg.drive.steering_angle = self.steering
        else:
            self.ack_msg.drive.steering_angle = self.steering

        self.ack_msg.header.stamp = rospy.Time.now()
        self.ack_msg.header.frame_id = "a"
        print("steering_angle", self.ack_msg.drive.steering_angle)
        self.pub.publish(self.ack_msg)

if __name__ == "__main__":
    rospy.init_node("ackermann_publisher", anonymous=True)

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        p = ackermann_publisher()
        print("Running")
        r.sleep()

