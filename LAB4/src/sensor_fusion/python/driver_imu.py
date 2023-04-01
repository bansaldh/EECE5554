#!/usr/bin/env python3

import rospy
import serial
import utm
from imu_driver.msg import Vectornav
#from imu_driver.srv import *
from imu_driver.srv import ConvertToQuaternion, ConvertToQuaternionResponse
from geometry_msgs.msg import Quaternion
import numpy as np
import re

serial_port = rospy.get_param('/imu_driver/port_2')

class ExampleNode:
	def __init__(self):
		rospy.init_node('imu_driver',anonymous=True)
		self.imu_publ = rospy.Publisher('imu',Vectornav,queue_size=10)
		self.s = rospy.Service('convert_to_quaternion',ConvertToQuaternion, self.handle_convert_to_quaternion)
		
	def imu_driver(self):
		#rospy.init_node("imu_driver", anonymous=True)
		#s = rospy.Service('convert_to_quaternion',ConvertToQuaternion, handle_convert_to_quaternion)
		
		#imu_publ = rospy.Publisher('imu', Vectornav, queue_size=10)
		msg1 = Vectornav()
	
		rate = rospy.Rate(40)        #We set the rate at 40Hz
	
		#The following line can be used if launch file is not to be used.
		#serial_port ='/dev/ttyUSB0' #Enter the serial port for which the RTK_GPS puck is connected.
		#If Launch were to be used.
		#serial_port = serial_port1
	
		serial_baud = 115200          #Ensure that the baud rate is set to 115200
		port = serial.Serial(serial_port,serial_baud,timeout=3.0)
	
		command = 'VNWRG,07,40\n'
		port.write(command.encode())
	
	
		while(not rospy.is_shutdown()):
		
			linesep1 = str(port.readline())  # To read each line from the IMU
			#rospy.loginfo(linesep1)
			data1 = linesep1.split("b'")     # Each Line starts with a b and hence to split it and remove it
			data2 = data1[1]                 # We want the remaining portion of the string as that contains the RTK_GPS data
			data3 = data2.split(",")         # Splitting the data by commas
			#rospy.loginfo(data3)
			#re.sub('[^\x00-\x7f]','',str(data3))
			#rospy.loginfo(data3)
	
			if "$VNYMR" in data3[0]:
				for i in range(13):
					print(data3[i])
				yaw    = float(data3[1])
				pitch  = float(data3[2])
				roll   = float(data3[3])
				magx   = float(data3[4])
				magy   = float(data3[5])
				magz   = float(data3[6])
				accelx = float(data3[7])
				accely = float(data3[8])
				accelz = float(data3[9])
				angx   = float(data3[10])
				angy   = float(data3[11])
	
				last_string = data3[12].split("*")
				angz   = float(last_string[0])
				
				try:
					rospy.wait_for_service('convert_to_quaternion')
					s = rospy.ServiceProxy('convert_to_quaternion',ConvertToQuaternion)
					result = s(roll, pitch, yaw)
					quaternion = result.quat
					print(quaternion)
				except rospy.ServiceException as e:
					print("Service Failed: %s"%e)
				
				
				msg1.header.frame_id="imu1_frame"
				msg1.header.stamp = rospy.Time.now()
				msg1.header.seq+=1
				
				msg1.imu.header.frame_id="imu1_frame"
				msg1.imu.header.stamp = rospy.Time.now()
				msg1.imu.header.seq+=1
				
				msg1.mag_field.header.frame_id="imu1_frame"
				msg1.mag_field.header.stamp = rospy.Time.now()
				msg1.mag_field.header.seq+=1
				
				msg1.imu.orientation.x = quaternion[0]
				msg1.imu.orientation.y = quaternion[1]
				msg1.imu.orientation.z = quaternion[2]
				msg1.imu.orientation.w = quaternion[3]
				
				msg1.imu.linear_acceleration.x = accelx
				msg1.imu.linear_acceleration.y = accely
				msg1.imu.linear_acceleration.z = accelz
				
				msg1.imu.angular_velocity.x = angx
				msg1.imu.angular_velocity.y = angy
				msg1.imu.angular_velocity.z = angz
				
				msg1.mag_field.magnetic_field.x = magx
				msg1.mag_field.magnetic_field.y = magy
				msg1.mag_field.magnetic_field.z = magz
				
				msg1.IMU_String = str(data2)
				
				#rospy.loginfo(msg1)
				self.imu_publ.publish(msg1)
				
				rate.sleep()
		rospy.spin()
		
		
	def handle_convert_to_quaternion(self,req):
		roll = req.roll 
		pitch = req.pitch
		yaw = req.yaw
		
		roll = np.radians(roll)
		pitch = np.radians(pitch)
		yaw = np.radians(yaw)

		# Perform quaternion conversion
		cy = np.cos(yaw * 0.5)
		sy = np.sin(yaw * 0.5)
		cp = np.cos(pitch * 0.5)
		sp = np.sin(pitch * 0.5)
		cr = np.cos(roll * 0.5)
		sr = np.sin(roll * 0.5)
		
		w = (cy * cp * cr) + (sy * sp * sr)
		x = (cy * cp * sr) - (sy * sp * cr)
		y = (sy * cp * sr) + (cy * sp * cr)
		z = (sy * cp * cr) - (cy * sp * sr)
		# Create response message
		quat = [x, y, z, w]
		
		return ConvertToQuaternionResponse(quat)
		
				
	
if __name__ == '__main__':
    try:    
    	node  = ExampleNode()
    	node.imu_driver()	
    	#imu_driver()
    except rospy.ROSInterruptException:
        pass
