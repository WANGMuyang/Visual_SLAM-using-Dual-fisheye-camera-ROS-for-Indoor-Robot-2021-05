#!/usr/bin/env python
import rospy
import sys
import time
from numpy.lib.scimath import sqrt
import msgpack
import matplotlib.pyplot as plt
import numpy as np
import yaml
import cv2
import math
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

def quaternion_to_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


class Plotter:
	def __init__(self, file_name, config_file_name):
		print("Initializing...")
		# Read config file
		with open(config_file_name) as file:
			self.cfg = yaml.load(file, Loader=yaml.FullLoader)

		# Read msg pack
		with open(file_name, "rb") as msg_pack_file:
		    msg_pack_byte_data = msg_pack_file.read()

		self.data = msgpack.unpackb(msg_pack_byte_data)

		self.image_row = 1000
		self.image_cols = 1000

		self.scale_integer = 30

		"""
		Extract keyframe poses
		"""
		print('Extracting keyframes ...')

		kfs_trans = []
		kfs_rot = []
		kfs_pos = []
		for kf in self.data['keyframes']:
			ktrans = self.data['keyframes'][kf]['trans_cw']
			kf_trans = [ktrans[0],ktrans[1],ktrans[2]]

			krot = self.data['keyframes'][kf]['rot_cw']
			kf_rot = [krot[0],krot[1],krot[2],krot[3]]

			kf_rot_mat = R.from_quat(kf_rot)
			kf_pos = np.matmul(-np.transpose(kf_rot_mat.as_matrix()), np.transpose(kf_trans))

			kfs_trans.append(kf_trans)
			kfs_rot.append(kf_rot)
			kfs_pos.append(kf_pos)

		self.keyframes_trans = np.array(kfs_trans)
		self.keyframes_rot = np.array(kfs_rot)
		self.keyframes_pos = np.array(kfs_pos) #  * 5x10^3

		# print(self.keyframes_pos)

		"""
		Extract stations poses
		"""
		print('Extracting stations ...')
		sts_pos = []
		if(len(self.data['stations'])!=0):
			for st in self.data['stations']:
				stpos = self.data['stations'][st]['pos_w']
				st_pos = [stpos[0],stpos[1],stpos[2]]
				sts_pos.append(st_pos)
				# print(st_pos)
		self.stations_pos = np.array(sts_pos)

		if(self.stations_pos.any()):
			self.rotate_stations(self.cfg['roll'],self.cfg['pitch'],self.cfg['yaw'])


		# 여기서 msgpack을 읽어서 keypoints 와 stations에 cfg에 적힌대로 회전변환 적용
		self.rotate_keypoints(self.cfg['roll'],self.cfg['pitch'],self.cfg['yaw'])

		# 이후 ros msg 를 받아서 callback함수를 돌려준다.
		# callback함수에서 현재 위치를 표시하고 관심 영역 지정, 스티어링 연산 및 2d map viewer 제공
		self.sub = rospy.Subscriber('/current_local', Pose, self.get_pose_and_heading_of_current_frame, queue_size=1)
		self.car_steering = 0
		self.pub = rospy.Publisher('ackermann_steering', Float32,queue_size=1)

	def get_pose_and_heading_of_current_frame(self, data): # 이게 중요한 함수임. 여기서 imshow 해준다.
		# imshow 할 때 잘 보이게 파라미터를 만들자

		# data가 현재 위치이며 ros msg 를 받은거임
		current_location_trans = []
		current_location_quaternion = []

		current_location_trans = [data.position.x,data.position.y,data.position.z]

		current_location_quaternion =  [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]

		current_location_quaternion_rot_mat = R.from_quat(current_location_quaternion)
		current_location_pos = np.matmul(-np.transpose(current_location_quaternion_rot_mat.as_matrix()), np.transpose(current_location_trans))

		self.current_location_trans = np.array(current_location_trans)
		self.current_location_quaternion = np.array(current_location_quaternion)
		self.current_location_pos = np.array(current_location_pos)

		# current_location에 cfg에 적힌대로 회전변환 적용
		self.rotate_current_location(self.cfg['roll'],self.cfg['pitch'],self.cfg['yaw'])

		# print('current_location_pos',self.current_location_pos)

		self.current_location_rot = quaternion_to_rotation_matrix(current_location_quaternion)

		self.current_location_rot_inv = self.transpose(self.current_location_rot)

		self.current_location_heading_vector = [self.current_location_rot_inv[0][2],
												self.current_location_rot_inv[1][2],self.current_location_rot_inv[2][2]]

		# current_location_heading에 cfg에 적힌대로 회전변환 적용
		self.rotate_current_location_heading(self.cfg['roll'],self.cfg['pitch'],self.cfg['yaw'])

		self.img = np.full((self.image_row, self.image_cols, 3), 0).astype(np.uint8) # 2d map 만들어서 볼 거임

		# draw keyframes_position
		for index in range(len(self.keyframes_pos)):
			cv2.circle(self.img,(int(-self.keyframes_pos[index][0]*self.scale_integer+self.image_row/2),
							int(self.keyframes_pos[index][1]*self.scale_integer+self.image_cols/2)),1,(255,255,255))

		# draw stations_position
		if(self.stations_pos.any()):
			for index in range(len(self.stations_pos)):
				cv2.circle(self.img,(int(-self.stations_pos[index][0]*self.scale_integer+self.image_row/2),
								int(self.stations_pos[index][1]*self.scale_integer+self.image_cols/2)),5,(255,255,0),-1)

		# draw current_position
		self.current_local_x = int(-self.current_location_pos[0]*self.scale_integer+self.image_row/2)
		self.current_local_y = int(self.current_location_pos[1]*self.scale_integer+self.image_cols/2)
		cv2.circle(self.img,(self.current_local_x,self.current_local_y),3,(0,255,0),-1)


		# draw heading = a vector from current_position(green) to red_circle is heading vector
		self.current_heading_x = int(-(self.current_location_pos[0]*self.scale_integer - self.current_location_heading_vector[0]*self.scale_integer)+self.image_row/2)
		self.current_heading_y = int((self.current_location_pos[1]*self.scale_integer + self.current_location_heading_vector[1]*self.scale_integer+self.image_cols/2))
		cv2.circle(self.img,(self.current_heading_x,self.current_heading_y),3,(0,0,255),-1)

		cv2.line(self.img,(self.current_local_x,self.current_local_y),(self.current_heading_x,self.current_heading_y),(0,0,255),2)

		#관심영역 설정 후 이 영역안의 keyframe위치를 따라가게 할 것
		if(self.stations_pos.any()):
			self.local_target_select_and_calculate_steering()

		cv2.imshow('a',self.img)
		cv2.waitKey(100) # wait for 100ms


	# 따라서 가도록 현재 바라보는 방향과 local target 이 맞도록 스티어링 연산
	def local_target_select_and_calculate_steering(self):
		# 현재 위치와 heading을 연결한 선의 기울기
		# 먼저 목표로 정할 keyframe을 정하기 위해 영역을 정할거임

		# 1. 기울기가 발산하는 경우(0으로 나누어짐)을 처리해줌
		if (int(self.current_heading_x - self.current_local_x) != 0):
			Slope = (self.current_heading_y - self.current_local_y)/(self.current_heading_x - self.current_local_x)
			# 2. 기울기가 0인 경우 제외
			if(Slope != 0):
				Reverse_Slope = -1/Slope

				width_of_roi = 30
				height_of_roi = 30
				local_width_slope = width_of_roi/Slope
				local_width_reverse_slope = height_of_roi/Reverse_Slope

				local_area_point1_x = int(self.current_local_x - Reverse_Slope*local_width_reverse_slope)
				local_area_point1_y = int(self.current_local_y - Reverse_Slope*local_width_reverse_slope)
				local_area_point1 = (local_area_point1_x,local_area_point1_y)

				local_area_point2_x = int(self.current_local_x + Slope*local_width_slope)
				local_area_point2_y = int(self.current_local_y + Slope*local_width_slope)
				local_area_point2 = (local_area_point2_x,local_area_point2_y)

				# cv2.circle(self.img,(local_area_point1_x,local_area_point1_y),5,(255,0,255),-1)
				# cv2.circle(self.img,(local_area_point2_x,local_area_point2_y),5,(255,0,255),-1)

				# local_area_point1 & local_area_point2가 영역 범위임
				cv2.rectangle(self.img,local_area_point1,local_area_point2,(0,255,255),2)

				station_x = (-self.stations_pos[0][0]*self.scale_integer+self.image_row/2)
				station_y = (self.stations_pos[0][1]*self.scale_integer+self.image_cols/2)

				most_minumun_differnece_index_of_keyframe = -1
				Difference = 1000000

				heading_local_vector = [self.current_heading_x - self.current_local_x, 
										self.current_heading_y - self.current_local_y]

				# secs = time.time()
				# print(secs)

				distance_between_current_local_and_station = (self.current_local_x - station_x)**2 + (self.current_local_y - station_y)**2

				for index in range(len(self.keyframes_pos)):
					keyframe_x = (-self.keyframes_pos[index][0]*self.scale_integer+self.image_row/2)
					keyframe_y = (self.keyframes_pos[index][1]*self.scale_integer+self.image_cols/2)
					distance_between_keyframe_and_station = (keyframe_x - station_x)**2 + (keyframe_y - station_y)**2

					# print("distance_between_current_local_and_station",distance_between_current_local_and_station)
					# print("distance_between_keyframe_and_station",distance_between_keyframe_and_station)

					# 관심영역 안에 존재하는 keyframes_pos에 대해
					if((local_area_point1_x < keyframe_x < local_area_point2_x) and (local_area_point1_y < keyframe_y < local_area_point2_y)):
						
						keyframe_local_vector =  [keyframe_x - self.current_local_x,keyframe_y - self.current_local_y]

						if(Difference > distance_between_keyframe_and_station):
							Difference  = distance_between_keyframe_and_station
							most_minumun_differnece_index_of_keyframe = index
						else:
							pass

				print(most_minumun_differnece_index_of_keyframe)

				if(most_minumun_differnece_index_of_keyframe>-1):
					# follow this keyframe
					keyframe_x = int(-self.keyframes_pos[most_minumun_differnece_index_of_keyframe][0]*self.scale_integer+self.image_row/2)
					keyframe_y = int(self.keyframes_pos[most_minumun_differnece_index_of_keyframe][1]*self.scale_integer+self.image_cols/2)
					cv2.circle(self.img,(keyframe_x,keyframe_y),5,(255,255,255),-1)

					local_keyframe_vector =  [keyframe_x - self.current_local_x, keyframe_y - self.current_local_y]

					theta = theta_between_two_vector(heading_local_vector, keyframe_local_vector)

					print("theta",theta)

					if(distance_between_current_local_and_station<5):
						print("stop")
						self.car_steering = 100
					else:
						self.car_steering = -(theta*math.pi/180) # 부호 반대임, radian으로 넣어준다
						self.car_steering = round(self.car_steering,2)
						print("car_steering",self.car_steering)
						self.pub.publish(self.car_steering)
					# secs = time.time()
					# print(secs)   # 모든 키프레임 위치에서 검사하면 0.0006초 정도 걸림



	def transpose(self,array):

		inv_array = np.zeros([3,3])

		inv_array[0][0] = array[0][0]
		inv_array[1][0] = array[0][1]
		inv_array[2][0] = array[0][2]

		inv_array[0][1] = array[1][0]
		inv_array[1][1] = array[1][1]
		inv_array[2][1] = array[1][2]

		inv_array[0][2] = array[2][0]
		inv_array[1][2] = array[2][1]
		inv_array[2][2] = array[2][2]

		return inv_array


	def rotation_matrix(self, axis, theta):
	    """
	    Return the rotation matrix associated with counterclockwise rotation about
	    the given axis by theta radians.
	    """
	    axis = np.asarray(axis)
	    axis = axis / np.sqrt(np.dot(axis, axis))
	    a = np.cos(theta / 2.0)
	    b, c, d = -axis * np.sin(theta / 2.0)
	    aa, bb, cc, dd = a * a, b * b, c * c, d * d
	    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
	    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
	                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
	                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


	def rotate(self, axis, angle, points):
		"""
		Rotate 3d coordinates
		"""
		rad_angle = (angle/180)*np.pi
		points = np.transpose(points)
		points = np.dot(self.rotation_matrix(axis, rad_angle), points)
		points = np.transpose(points)
		return points


	def rotate_stations(self, x_angle, y_angle, z_angle):

		print('Applying rotation to stations...')

		self.stations_pos = self.rotate([1,0,0],x_angle,self.stations_pos)

		self.stations_pos = self.rotate([0,1,0],y_angle,self.stations_pos)

		self.stations_pos = self.rotate([0,0,1],z_angle,self.stations_pos)



	def rotate_keypoints(self, x_angle, y_angle, z_angle):

		print('Applying rotation to keypoints ...')

		self.keyframes_pos = self.rotate([1,0,0],x_angle,self.keyframes_pos)

		self.keyframes_pos = self.rotate([0,1,0],y_angle,self.keyframes_pos)

		self.keyframes_pos = self.rotate([0,0,1],z_angle,self.keyframes_pos)


	def rotate_current_location(self, x_angle, y_angle, z_angle):

		self.current_location_pos = self.rotate([1,0,0],x_angle,self.current_location_pos)

		self.current_location_pos = self.rotate([0,1,0],y_angle,self.current_location_pos)

		self.current_location_pos = self.rotate([0,0,1],z_angle,self.current_location_pos)


	def rotate_current_location_heading(self, x_angle, y_angle, z_angle):

		self.current_location_heading_vector = self.rotate([1,0,0],x_angle,self.current_location_heading_vector)

		self.current_location_heading_vector = self.rotate([0,1,0],y_angle,self.current_location_heading_vector)

		self.current_location_heading_vector = self.rotate([0,0,1],z_angle,self.current_location_heading_vector)

def theta_between_two_vector(vec1,vec2): # heading_local // keyframe_loacl
	theta2 = math.atan(float(vec2[1]/vec2[0]))
	theta1 = math.atan(float(vec1[1]/vec1[0]))
	theta = theta2-theta1
	format(theta, ".2f")
	# heading_local을 x축이라 보고 theta의 부호를 정한다.
	return theta


if __name__ == "__main__":
	rospy.init_node("plot_2d_map", anonymous=True)

	if len(sys.argv) != 3:
		print('Implementation:  python3 plotter.py /path/to/msgpack/file.msg /home/alpha/openvslam_occ_grid-master/config.yaml')
		sys.exit()

	p = Plotter(sys.argv[1],sys.argv[2])

	rospy.spin()





