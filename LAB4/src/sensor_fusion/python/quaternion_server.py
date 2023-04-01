#!/usr/bin/env python3

import rospy
import serial
import utm
from imu_driver.msg import Vectornav
from imu_driver.srv import ConvertToQuaternion, ConvertToQuaternionResponse
from geometry_msgs.msg import Quaternion
import numpy as np


def handle_convert_to_quaternion(req):
	roll = req.roll
	pitch = req.pitch
	yaw = req.yaw
	# Perform quaternion conversion
	cy = np.cos(yaw * 0.5)
	sy = np.sin(yaw * 0.5)
	cp = np.cos(pitch * 0.5)
	sp = np.sin(pitch * 0.5)
	cr = np.cos(roll * 0.5)
	sr = np.sin(roll * 0.5)
	
	w = cy * cp * cr + sy * sp * sr
	x = cy * cp * sr - sy * sp * cr
	y = sy * cp * sr + cy * sp * cr
	z = sy * cp * cr - cy * sp * sr
	# Create response message
	quat = [x, y, z, w]
	
	return ConvertToQuaternionResponse(quat)
	
			
def convert_to_quaternion_server():
	rospy.init_node('convert_to_quaternion_server')
	s = rospy.Service('convert_to_quaternion',ConvertToQuaternion, handle_convert_to_quaternion)	
	print('Reached Quaternion server')
	rospy.spin()

			
	
if __name__ == '__main__':
    try:    	
    	convert_to_quaternion_server()
    except rospy.ROSInterruptException:
        pass
