#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int32MultiArray
import geometry_msgs.msg
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
import tf2_ros
import time
import json
from simple_pid import PID

# Physical params
robot_length = 0.42
robot_width = 0.35
from_back_to_lidar = 0.39

# Behavior params
stop_dist = 0.25
avoid_length = 0.50 # 1.0
avoid_width = 0.75

#									Y
#									|					             								
#									|								 |<-avoid_length->|
#								|	|                   |<-stop_dist>|                .
#								|<-----robot_length---->|            .                .
#								|	|                   |            .                .
#	P---------------------------M	|					G-----------------------------I		<-
#	.			   .			|	|					|			 .				  .	     |
#	.			   L------------A___|___________________B------------E				  .      |
#	.			   .        	|	|		^			|  	 		 .				  .      |
#	.			   .	    	|   |		|			|            .                .      |
#	.			   .      		|   |		|			|      	     .				  .      |
#	.			   .    		|	|		|			|            .	              .      | avoid_width
#	.			   .  			|	|	robot_width		|	         .                .      |
#	S--------------W------------|---O-------------------|------------T----------------K------|------- X
#	.			   . 			|	|		|			|   	     .                .      |
#	.			   .    		|	|		|			|		     .                .      |
#	.			   .      		|	|		|			| 	         .                .      |
#	.			   .        	|	|		|			|            .                .      | 
#	.			   .		 	|	|		|			|            .                .      |
#	.			   R------------C___|_______V___________D------------F                .      |
#	.			   .			|	|					|            .                .      |
#	Q---------------------------N	|					H-----------------------------J     <-
#
# Robot's footprint is ABCD
# O is where the lidar is placing
# robot is facing forward on X-direction
# BDT and ACW are the STOP_ZONE

# AB is robot length
# BD is robot width
# AC_O is lidar offset from back side

AB = robot_length
BD = robot_width
AC_O = from_back_to_lidar

# Footprint coords
A = (-AC_O, robot_width/2)
B = (robot_length-AC_O, robot_width/2)
C = (-AC_O, -robot_width/2)
D = (robot_length-AC_O, -robot_width/2)
# Front Stop and Avoid zone coords
G = ((robot_length-AC_O), avoid_width/2)
H = ((robot_length-AC_O), -avoid_width/2)
E = ((robot_length-AC_O+stop_dist), robot_width/2)
F = ((robot_length-AC_O+stop_dist), -robot_width/2)
I = ((robot_length-AC_O+stop_dist+avoid_length), avoid_width/2)
J = ((robot_length-AC_O+stop_dist+avoid_length), -avoid_width/2)
T = ((robot_length-AC_O+stop_dist), 0)
K = ((robot_length-AC_O+stop_dist+avoid_length), 0)
# Back Stop and Avoid zone coords
M = (-AC_O, avoid_width/2)
N = (-AC_O, -avoid_width/2)
L = ((-AC_O-stop_dist), robot_width/2)
R = ((-AC_O-stop_dist), -robot_width/2)
P = ((-AC_O-stop_dist-avoid_length), avoid_width/2)
Q = ((-AC_O-stop_dist-avoid_length), -avoid_width/2)
W = ((-AC_O-stop_dist), 0)
S = ((-AC_O-stop_dist-avoid_length), 0)


## Stop zone of triangle shape
## Front
# slope and y-intercept of first quadant
m1 = (T[1] - B[1])/(T[0] - B[0])
b1 = B[1] - m1*(B[0])
# slope and y-intercept of second quadant
m2 = (T[1] - D[1])/(T[0] - D[0])
b2 = D[1] - m2*(D[0])

## Back
# slope and y-intercept of thrid quadant
m3 = (A[1] - W[1])/(A[0] - W[0])
b3 = A[1] - m3*(A[0])
# slope and y-intercept of fourth quadant
m4 = (C[1] - W[1])/(C[0] - W[0])
b4 = C[1] - m4*(C[0])



print("A", A)
print("B", B)
print("C", C)
print("D", D)
print("E", E)
print("F", F)
print("G", G)
print("H", H)
print("I", I)
print("J", J)
print("K", K)
print("L", L)
print("M", M)
print("N", N)
print("P", P)
print("Q", Q)
print("R", R)
print("S", S)
print("T", T)
print("W", W)
# quit()

class DistKeeperNav:

	def __init__(self):

		rospy.init_node("distances_keep_nav_node", anonymous=True)
		rospy.Subscriber("/scan", LaserScan, self.scan_callback)
		rospy.Subscriber("/jmoab_imu_raw", Imu, self.imu_callback)

		# self.image_pub = rospy.Publisher("image_topic", Image)
		# self.bw_image_pub = rospy.Publisher("bw_image_topic", Image)
		# self.bridge = CvBridge()

		self.left_wall_scan_pub = rospy.Publisher("/left_wall_scan", LaserScan, queue_size=1)
		self.right_wall_scan_pub = rospy.Publisher("/right_wall_scan", LaserScan, queue_size=1)
		self.left_wall_scan = LaserScan()
		self.right_wall_scan = LaserScan()

		self.sbus_cmd_pub = rospy.Publisher("/sbus_cmd", Int32MultiArray, queue_size=10)
		self.sbus_cmd = Int32MultiArray()

		self.foot_pub = rospy.Publisher("/footprint", PolygonStamped)
		self.front_stop_pub = rospy.Publisher("/front_stop_zone", PolygonStamped)
		self.back_stop_pub = rospy.Publisher("/back_stop_zone", PolygonStamped)

		self.pg = PolygonStamped()
		self.pg_front_stop = PolygonStamped()
		self.pg_back_stop = PolygonStamped()

		self.br = tf2_ros.TransformBroadcaster()
		self.t = geometry_msgs.msg.TransformStamped()

		########################### Image Parameters ##########################
		##### Frame size #####
		self.frame_width = 300
		self.frame_height = 300 
		self.frame_width_spacing = 140	#250
		self.frame_height_spacing = 140	#150

		self.frame_real_width = float(self.frame_width/self.frame_width_spacing)
		self.frame_real_height = float(self.frame_height/self.frame_height_spacing)

		##### Blank map #####
		self._map = np.ones((self.frame_height, self.frame_width, 3), np.uint8) * 255
		self.amap = np.copy(self._map)

		########################### Wall scan repeater ##########################
		self.ranges_list = None
		self.angle_list = None
		self.angle_min = None
		self.angle_max = None
		self.angle_increment = None
		self.intensities = None


		### Wall detect params
		## Left
		self.left_wall_detect_deg = 15.0
		pcs_on_pie = 1.0/(self.left_wall_detect_deg/360.0)
		self.shifted_from_mid = int(2019.0/pcs_on_pie)		# ranges_len is constant as 2019
		self.leftWall_mid_idx = 1515 				# left side, 3/4 of 2019
		self.leftWall_left_shifted_idx = self.leftWall_mid_idx - self.shifted_from_mid
		self.leftWall_right_shifted_dix = self.leftWall_mid_idx + self.shifted_from_mid
		## Right 
		self.right_wall_detect_deg = self.left_wall_detect_deg
		self.rightWall_mid_idx = 505 	# 1/4 of 2019
		self.rightWall_left_shifted_idx = self.rightWall_mid_idx - self.shifted_from_mid
		self.rightWall_right_shifted_idx = self.rightWall_mid_idx + self.shifted_from_mid

		########################### Robot Parameters ##########################
		self.sbus_steering_mid = 1024
		self.sbus_throttle_mid = 1031	#1024

		## Throttle
		self.sbus_throttle_fwd_const = self.sbus_throttle_mid + 59	# =1090
		self.sbus_throttle_bwd_const = self.sbus_throttle_mid - 59	# =972

		self.sbus_throttle_adj = 12
		self.sbus_throttle_fwd_max = self.sbus_throttle_fwd_const
		self.sbus_throttle_fwd_min = self.sbus_throttle_fwd_const - self.sbus_throttle_adj
		self.sbus_throttle_bwd_max = self.sbus_throttle_bwd_const
		self.sbus_throttle_bwd_min = self.sbus_throttle_bwd_const + self.sbus_throttle_adj

		## Steering
		self.sbus_steering_adj = 12	#31	
		self.sbus_steering_max_DB = 1044	# 1024+24  the lowest value of steering to the left
		self.sbus_steering_min_DB = 996	# 1024-24  the lowest value of steering to the right
		self.sbus_steering_max = self.sbus_steering_max_DB + self.sbus_steering_adj
		self.sbus_steering_min = self.sbus_steering_min_DB - self.sbus_steering_adj

		self.sbus_steering_BWD_max_DB = 1052	# 1024+24  the lowest value of steering to the left
		self.sbus_steering_BWD_min_DB = 1004	# 1024-24  the lowest value of steering to the right
		self.sbus_steering_BWD_max = self.sbus_steering_BWD_max_DB + self.sbus_steering_adj
		self.sbus_steering_BWD_min = self.sbus_steering_BWD_min_DB - self.sbus_steering_adj

		########################### PID Parameters ##########################
		## PID distance keeping
		self.diff_thresh = 0.05
		self.kp = 15.0		# 15.0
		self.ki = 0.0
		self.kd = 0.00
		self.setpoint = 0.0

		self.pid = PID(self.kp, self.ki, self.kd, setpoint=self.setpoint)
		self.pid.tunings = (self.kp, self.ki, self.kd)
		self.pid.sample_time = 0.001
		self.pid.output_limits = (-1.0, 1.0)
		

		## PID wall follow
		self.kp_wf = 30.0
		self.ki_wf = 0.0
		self.kd_wf = 0.00
		self.setpoint_wf = 0.35

		self.pid_wf = PID(self.kp_wf, self.ki_wf, self.kd_wf, setpoint=self.setpoint_wf)
		self.pid_wf.tunings = (self.kp_wf, self.ki_wf, self.kd_wf)
		self.pid_wf.sample_time = 0.001
		self.pid_wf.output_limits = (-1.0, 1.0)
		self.pid_wf.auto_mode = False


		##################### Robot Stop zone ########################
		self.FRONT_STOP = False
		self.BACK_STOP = False

		self.ROBOT_MODE = 'FRONT'
		self.prev_ROBOT_MODE = self.ROBOT_MODE

		self.X = np.array([])
		self.Y = np.array([])
		self.PX = np.array([])
		self.PY = np.array([])

		self.hdg = None

		self.run()

		rospy.spin()

	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter


		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		# print(m, val, in_min, in_max, out_min, out_max)

		return out

	def inFrontStopZone(self, x,y):

		if (B[0] < x) and (x < T[0]):
			if (D[1] < y) and (y < B[1]):
				y_cal1 = m1*x + b1
				y_cal2 = m2*x + b2
				# print("y_cal1", y_cal1)
				# print("y_cal2", y_cal2)
				if (y < y_cal1) and (y > y_cal2):
					return True
				else:
					return False

			else:
				return False

		else:
			return False

	def inBackStopZone(self, x, y):

		if (W[0] < x) and (x < A[0]):
			if (C[1] < y) and (y < A[1]):

				y_cal3 = m3*x + b3
				y_cal4 = m4*x + b4

				if (y < y_cal3) and (y > y_cal4):
					return True
				else:
					return False
			else:
				return False
		else:
			return False

	def scan_callback(self, msg):
		self.ranges_list = msg.ranges
		self.angle_min = msg.angle_min
		self.angle_max = msg.angle_max
		self.angle_increment = msg.angle_increment
		self.received_time = time.time()	#rospy.Time.now()
		self.intensities = msg.intensities

		self.angle_list = np.arange((self.angle_min), self.angle_max, self.angle_increment)

		## copy original scan msg to wall_scan msg
		## and publish wall_scan topic immediately
		self.left_wall_scan.header.stamp = rospy.Time.now()
		self.left_wall_scan.header.frame_id = "laser_frame"
		self.left_wall_scan.time_increment = msg.time_increment
		self.left_wall_scan.angle_increment = msg.angle_increment
		self.left_wall_scan.angle_min = ((90-self.left_wall_detect_deg)*pi)/180.0
		self.left_wall_scan.angle_max = ((90+self.left_wall_detect_deg)*pi)/180.0
		self.left_wall_scan.scan_time = msg.scan_time
		self.left_wall_scan.range_min = msg.range_min
		self.left_wall_scan.range_max = 5.0	#msg.range_max
		self.left_wall_scan.ranges = msg.ranges[self.leftWall_left_shifted_idx:self.leftWall_right_shifted_dix]
		self.left_wall_scan.intensities = msg.intensities[self.leftWall_left_shifted_idx:self.leftWall_right_shifted_dix]

		self.right_wall_scan.header.stamp = rospy.Time.now()
		self.right_wall_scan.header.frame_id = "laser_frame"
		self.right_wall_scan.time_increment = msg.time_increment
		self.right_wall_scan.angle_increment = msg.angle_increment
		self.right_wall_scan.angle_min = ((-90-self.right_wall_detect_deg)*pi)/180.0
		self.right_wall_scan.angle_max = ((-90+self.right_wall_detect_deg)*pi)/180.0
		self.right_wall_scan.scan_time = msg.scan_time
		self.right_wall_scan.range_min = msg.range_min
		self.right_wall_scan.range_max = 5.0	#msg.range_max
		self.right_wall_scan.ranges = msg.ranges[self.rightWall_left_shifted_idx:self.rightWall_right_shifted_idx]
		self.right_wall_scan.intensities = msg.intensities[self.rightWall_left_shifted_idx:self.rightWall_right_shifted_idx]



		## if I put this in the run() loop, I will get strange value of x,y sometimes...
		self.X = np.asarray(self.ranges_list, dtype=np.float)*np.cos(np.asarray(self.angle_list, dtype=np.float))
		self.Y = np.asarray(self.ranges_list, dtype=np.float)*np.sin(np.asarray(self.angle_list, dtype=np.float))

		self.PX = self.X*self.frame_width_spacing + self.frame_width/2
		self.PY = self.Y*self.frame_height_spacing + self.frame_height/2

		# if self.inFrontStopZone(self.X, self.Y):
		# 	print("Stopp")

		## this for loop takes ~5ms
		for i, (x,y) in enumerate(zip(self.X, self.Y)):
			if self.inFrontStopZone(x,y):
				# print("In FRONT STOP zone: i: {:d}  x {:.7f} y {:.7f}  | range {:.3f}  ang {:.3f}".format(i,x,y, self.ranges_list[i], self.angle_list[i]))
				_front_stop_flag = True
				break
			else:
				_front_stop_flag = False

			# if self.inBackStopZone(x, y):
			# 	# print("In BACK STOP zone:  x_in {:.4f} y_in {:.4f}".format(x_in,y_in))
			# 	self.BACK_STOP = True
			# 	break
			# else:
			# 	self.BACK_STOP = False

		## We set FRONT_STOP flag only after finish the for loop
		self.FRONT_STOP = _front_stop_flag

	def image_bitmap(self, px, py):
		if px > 0 and px < self.frame_width:
			if py > 0 and py < self.frame_height:
				self.amap[py,px] = 0,0,0


	def turnRight90Deg(self, start_imu_deg, current_imu_deg):
		## This target deg comes from experiment
		target_deg = 75.0
		goal_imu_deg = start_imu_deg - target_deg

		if (goal_imu_deg < -180):
			current_imu_deg = current_imu_deg%360
			goal_imu_deg = goal_imu_deg%360

		diff = current_imu_deg - goal_imu_deg

		if (diff > 0.0):
			return [928, 1040], False
		else:
			return [1024, 1024], True

	def turnRight180Deg(self, start_imu_deg, current_imu_deg):
		## This target deg comes from experiment
		target_deg = 167.0
		goal_imu_deg = start_imu_deg - target_deg

		if (goal_imu_deg < -180):
			current_imu_deg = current_imu_deg%360
			goal_imu_deg = goal_imu_deg%360

		diff = current_imu_deg - goal_imu_deg

		if (diff > 0.0):
			return [928, 1040], False
		else:
			return [1024, 1024], True

	def turnLeft90Deg(self, start_imu_deg, current_imu_deg):
		## This target deg comes from experiment
		target_deg = 76.0
		goal_imu_deg = start_imu_deg + target_deg

		if (goal_imu_deg > 180):
			## convert current_deg to 0-360 range
			current_imu_deg = current_imu_deg%360
			goal_imu_deg = goal_imu_deg%360

		diff = goal_imu_deg - current_imu_deg
		if diff > 0.0:
			return [1120, 1034], False
		else:
			return [1024, 1024], True

	def turnLeft180Deg(self, start_imu_deg, current_imu_deg):
		## This target deg comes from experiment
		target_deg = 167.0
		goal_imu_deg = start_imu_deg + target_deg

		if (goal_imu_deg > 180):
			## convert current_deg to 0-360 range
			current_imu_deg = current_imu_deg%360
			goal_imu_deg = goal_imu_deg%360

		diff = goal_imu_deg - current_imu_deg
		if diff > 0.0:
			return [1120, 1034], False
		else:
			return [1024, 1024], True

	def imu_callback(self, data):

		qw = data.orientation.w
		qx = data.orientation.x
		qy = data.orientation.y
		qz = data.orientation.z

		r11 = qw**2 + qx**2 - qy**2 - qz**2 #1 - 2*qy**2 - 2*qz**2
		r12 = 2*qx*qy - 2*qz*qw
		r13 = 2*qx*qz + 2*qy*qw
		r21 = 2*qx*qy + 2*qz*qw
		r22 = qw**2 - qx**2 + qy**2 - qz**2	#1 - 2*qx**2 - 2*qz**2
		r23 = 2*qy*qz - 2*qx*qw
		r31 = 2*qx*qz - 2*qy*qw
		r32 = 2*qy*qz + 2*qx*qw
		r33 = qw**2 - qx**2 - qy**2 + qz**2	#1 - 2*qx**2 - 2*qy**2
		rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

		## default orientation (heatsink is pointing down)
		rot_matrix = np.array([
								[ 1, 0, 0],
								[ 0, 1, 0],
								[ 0, 0, 1]])

		rot = np.dot(rot,rot_matrix)

		yaw2 = np.arctan2(rot[1,0],rot[0,0])
		pitch = np.arcsin(-rot[2,0])
		roll = np.arctan2(rot[2,1],rot[2,2])

		self.hdg = np.degrees(yaw2)

	def run(self):
		rate = rospy.Rate(20) # 10hz

		prev_diff = 0.0
		done_turn_flag = False

		while not rospy.is_shutdown():

			if self.left_wall_scan.ranges and self.right_wall_scan.ranges:
				left_wall_ranges_array = np.asarray(self.left_wall_scan.ranges)
				right_wall_ranges_array = np.asarray(self.right_wall_scan.ranges)
				## remove zeros out from array
				left_wall_ranges_array = left_wall_ranges_array[left_wall_ranges_array != 0]
				right_wall_ranges_array = right_wall_ranges_array[right_wall_ranges_array != 0]

				left_wall_ranges_min = np.min(left_wall_ranges_array)
				right_wall_ranges_min = np.min(right_wall_ranges_array)

				if self.ROBOT_MODE == "FRONT":
					if self.FRONT_STOP:
						self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]
						stop_timeout = time.time() - tic
						print("Stop with time {:.3f}".format(stop_timeout))

						if stop_timeout > 3.0:
							self.ROBOT_MODE = "UTURN"
							start_ang = self.hdg		# capture starting angle before turning
							done_turn_flag = False		# set turning flag to False
							self.pid.auto_mode = False	# turn off pid
					else:
						tic = time.time()
					
						diff = left_wall_ranges_min - right_wall_ranges_min
						output_pid = self.pid(diff)
						sbus_steering = int(self.map_with_limit(output_pid, -1.0, 1.0, self.sbus_steering_max, self.sbus_steering_min))
						sbus_throttle = self.sbus_throttle_fwd_const
							

						self.sbus_cmd.data = [sbus_steering, sbus_throttle]
						print("min_left: {:.3f} | min_right: {:.3f} | diff: {:.3f} | output_pid: {:.2f} | str: {:d} | thr: {:d}".format(\
								left_wall_ranges_min, right_wall_ranges_min, diff, output_pid, sbus_steering, sbus_throttle))

						if left_wall_ranges_min > 1.0 or right_wall_ranges_min > 1.0:
							self.ROBOT_MODE = "LEFT_FOLLOW"
							self.pid.auto_mode = False		# disable distance keeping PID
							self.pid_wf.auto_mode = True 	# enable wall following PID
							self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]


				elif self.ROBOT_MODE == "UTURN":

					if not done_turn_flag:
						print("hdg: {:.3f} | start_ang: {:.3f} ".format(self.hdg, start_ang))
						self.sbus_cmd.data, done_turn_flag = self.turnLeft180Deg(start_ang, self.hdg)
					
					if done_turn_flag:
						self.pid.auto_mode = True
						self.ROBOT_MODE = "FRONT"

				elif self.ROBOT_MODE == "LEFT_FOLLOW":
					print("start wall follow mode")

					output_pid_wf = self.pid_wf(left_wall_ranges_min)
					sbus_steering = int(self.map_with_limit(output_pid_wf, -1.0, 1.0, self.sbus_steering_max, self.sbus_steering_min))
					sbus_throttle = self.sbus_throttle_fwd_const
					print("min_left: {:.3f} | output_pid_wf: {:.2f} | str: {:d} | thr: {:d}".format(\
								left_wall_ranges_min, output_pid_wf, sbus_steering, sbus_throttle))

					self.sbus_cmd.data = [sbus_steering, sbus_throttle]

					# wf_tic = time.time()

					if left_wall_ranges_min < 1.0 and right_wall_ranges_min < 1.0:
						# wf_timeout = time.time() - wf_tic
						# if wf_timeout > 1.0:
						self.ROBOT_MODE = "FRONT"
						self.pid_wf.auto_mode = False
						self.pid.auto_mode = True




			else:
				self.sbus_cmd.data = [self.sbus_steering_mid, self.sbus_throttle_mid]


			# prev_diff = diff
			self.left_wall_scan_pub.publish(self.left_wall_scan)
			self.right_wall_scan_pub.publish(self.right_wall_scan)

			self.sbus_cmd_pub.publish(self.sbus_cmd)


			## Drawing Polygons ##
			# footprint
			self.pg.header.stamp = rospy.Time.now()
			self.pg.header.frame_id = "base_footprint"
			self.pg.polygon.points = [
								Point32(x=A[0], y=A[1]),
								Point32(x=B[0], y=B[1]),
								Point32(x=D[0], y=D[1]),
								Point32(x=C[0], y=C[1])]

			# front stop zone
			self.pg_front_stop.header.stamp = rospy.Time.now()
			self.pg_front_stop.header.frame_id = "base_footprint"
			self.pg_front_stop.polygon.points = [
								Point32(x=B[0], y=B[1]),
								Point32(x=T[0], y=T[1]),
								Point32(x=D[0], y=D[1]),
								Point32(x=B[0], y=B[1])]

			# back stop zone
			self.pg_back_stop.header.stamp = rospy.Time.now()
			self.pg_back_stop.header.frame_id = "base_footprint"
			self.pg_back_stop.polygon.points = [
									Point32(x=A[0], y=A[1]),
									Point32(x=C[0], y=C[1]),
									Point32(x=W[0], y=W[1]),
									Point32(x=A[0], y=A[1])]

			# construct tf
			self.t.header.frame_id = "base_footprint" 
			self.t.header.stamp = rospy.Time.now()
			self.t.child_frame_id = "base_link"
			self.t.transform.translation.x = 0.0
			self.t.transform.translation.y = 0.0
			self.t.transform.translation.z = 0.0

			self.t.transform.rotation.x = 0.0
			self.t.transform.rotation.y = 0.0
			self.t.transform.rotation.z = 0.0
			self.t.transform.rotation.w = 1.0
			self.br.sendTransform(self.t)

			# laser_pub.publish(new_scan)
			self.foot_pub.publish(self.pg)
			self.front_stop_pub.publish(self.pg_front_stop)
			self.back_stop_pub.publish(self.pg_back_stop)



			rate.sleep()




if __name__ == "__main__":

	dkn = DistKeeperNav()