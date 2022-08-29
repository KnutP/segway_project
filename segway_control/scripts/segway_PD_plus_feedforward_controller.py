#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, atan2, cos, sin
import numpy as np
import rospy, tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from gazebo_msgs.srv import GetModelState
import matplotlib.pyplot as plt

class Segway():
	def __init__(self):
		rospy.init_node("turtlebot_move")
		rospy.loginfo("Press Ctrl + C to terminate")
		self.leftTorque = 0
		self.rightTorque = 0

		self.lWheelPub = rospy.Publisher("/segway/joint1_effort_controller/command", Float64, queue_size=10)
		self.rWheelPub = rospy.Publisher("/segway/joint2_effort_controller/command", Float64, queue_size=10)
        
		self.rate = rospy.Rate(10)

		self.pose = Pose2D()
		self.trajectory = list()
		rospy.loginfo("created traj")

		rospy.wait_for_service("/gazebo/get_model_state")
		self.gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
		resp = self.gms("m2wr", "")
		quarternion = [resp.pose.orientation.x,resp.pose.orientation.y,resp.pose.orientation.z, resp.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (quarternion)
		self.pose.theta = pitch
		self.heading = yaw
		self.pose.x = resp.pose.position.x
		self.pose.y = resp.pose.position.y

		self.last_pos = self.pose.x
		self.last_angle = self.pose.theta
		self.last_heading = self.heading
		self.is_at_position = False
		self.t = 0
		
		try:
			self.run()
		except rospy.ROSInterruptException:
			rospy.loginfo("Action terminated.")
		finally:
			# save trajectory into csv file
			rospy.loginfo("Sending csv")
			np.savetxt('trajectory.csv', np.array(self.trajectory), delimiter=',', fmt='%f')
			_, ax = plt.subplots(1)
			ax.set_aspect('equal')

			trajectory = np.loadtxt("trajectory.csv", delimiter=',')
			# plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)
			# plt.xlim(-1, 5)
			# plt.ylim(-1, 5)
			# plt.show()

			plt.plot(trajectory[:, 3], trajectory[:, 2], linewidth=2)
			plt.xlim(0, self.t+0.2)
			plt.ylim(-1, 2)
			plt.title("Segway angle vs time")
			plt.xlabel("Time (s)")
			plt.ylabel("Theta (rad)")
			plt.show()


	def run(self):
		while not rospy.is_shutdown():
			resp = self.gms("m2wr", "")
			quarternion = [resp.pose.orientation.x,resp.pose.orientation.y,resp.pose.orientation.z, resp.pose.orientation.w]
			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (quarternion)
			self.pose.theta = pitch
			self.heading = yaw
			self.pose.x = resp.pose.position.x
			self.pose.y = resp.pose.position.y
			dtheta = (self.pose.theta-self.last_angle)/0.005

			# Set desired values for Segway angle, heading, and x position
			des_angle = 0
			des_position = 4
			des_heading = 0.5

			if self.pose.x > des_position-0.2 and self.pose.x < self.last_pos:
				self.is_at_position = True

			# PD controller for moving the robot to the desired position
			kp_pos = 0.045
			kd_pos = 0.001
			pos_torque = kp_pos*(des_position-self.pose.x) +-kd_pos*(self.pose.x-self.last_pos)/0.005

			if self.is_at_position:
				pos_torque = 0

			# PD controller for keeping the robot facing toward the desired heading
			kp_heading = 0.3
			kd_heading = 0.0054
			heading_torque = kp_heading*(des_heading-self.heading) - kd_heading*(self.heading - self.last_heading)/0.005

			#### Model Parameters ####
			m_b = 1
			m = 0.1
			r = 0.2
			I_w = 0.001
			I_b = 0.001
			L = 0.2
			g = 9.8

			M = m*L*r*cos(self.pose.theta)
			C = -m*0*L*r*sin(self.pose.theta)*dtheta
			G = m*g*L*sin(self.pose.theta)*0.01
			ddtheta_d = 0

			# Computed Torque controller for keeping the segway upright
			# Main controller we are testing
			kp_angle = 0.01
			kd_angle = 0.0003
			angle_torque = kp_angle*(des_angle-self.pose.theta) - kd_angle*dtheta + M*ddtheta_d + C*dtheta + G

			# Combine inputs from all three controllers
			leftTorque = - angle_torque + pos_torque - heading_torque
			rightTorque = - angle_torque + pos_torque + heading_torque

			self.last_pos = self.pose.x
			self.last_angle = self.pose.theta
			self.last_heading = self.heading
			
			self.trajectory.append([self.pose.x, self.pose.y, self.pose.theta, self.t])
			self.t += 0.1

			self.lWheelPub.publish(Float64(leftTorque))
			self.rWheelPub.publish(Float64(rightTorque))


			rospy.loginfo("Left torque: " + str(leftTorque) + " Right torque: " + str(rightTorque))
			rospy.loginfo("Position torque: " + str(pos_torque) + " Angle torque: " + str(angle_torque))
			rospy.loginfo("Angle: " + str(self.pose.theta) + " Heading: " + str(self.heading))
			rospy.loginfo("x: " + str(self.pose.x) + " y: " + str(self.pose.y))
			self.rate.sleep()


if __name__ == '__main__':
	whatever = Segway()