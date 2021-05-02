#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image_publisher():
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.pub = rospy.Publisher('camera/image_raw', Image, queue_size=1)
    
    def image_cb(self):
        img = cv2.imread("/home/alpha/single_image_publisher/station.png",1)
        img_msg = self.cv_bridge.cv2_to_imgmsg(img,encoding="bgr8")
        self.pub.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node("single_image_publisher", anonymous=True)
    single = image_publisher()
    a=1
    while(a<200):
        a+=1
        single.image_cb()


						# keyframe_heading_vector를 만들어주고
						keyframe_heading_vector =  [keyframe_x - self.current_local_x,keyframe_y - self.current_local_y]
						if(index == 0):
							# current_heading 이거 필요해?
							# 또한 stations_pos부터 current_heading, current_local까지의 거리를 통해 추가적인 범위를 준다.
							# 이 경우 station과 current_local의 거리보다 station과 keyframe의 거리가 작아야함
							if(distance_between_current_local_and_station > self.minumum_distance_between_keyframe_and_station):
								if(distance_between_keyframe_and_station  > distance_between_current_local_and_station):
									# print(theta_between_two_vector(Direction_vector,keyframe_heading_vector))
									Difference  = abs(theta_between_two_vector(Direction_vector,keyframe_heading_vector))
								else:
									pass
							else:
								if(distance_between_current_local_and_station > distance_between_keyframe_and_station):
										Difference  = abs(theta_between_two_vector(Direction_vector,keyframe_heading_vector))
								else:
									pass
						else:
							if(distance_between_heading_and_station > distance_between_current_local_and_station):
								if(distance_between_keyframe_and_station  > distance_between_current_local_and_station):
									if(Difference  > abs(theta_between_two_vector(Direction_vector,keyframe_heading_vector))):
										most_minumun_differnece_index_of_keyframe = index
									else:
										pass
							else:
								if(distance_between_current_local_and_station > distance_between_keyframe_and_station):
									if(Difference  > abs(theta_between_two_vector(Direction_vector,keyframe_heading_vector))):
										most_minumun_differnece_index_of_keyframe = index
									else:
										pass