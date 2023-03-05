#!/usr/bin/env python3

import sys
import rospy
import serial
import numpy as np
import roslib
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from imu_driver.msg import Vectornav
from imu_driver.srv import conversion

def convert_to_quaternion(r, p ,y):
	r = np.deg2rad(r)
	p = np.deg2rad(p)
	y = np.deg2rad(y)
	qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)

	qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)

	qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
	
	qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
	return[qw,qx,qy,qz]	
                
def convert_service(req):
	r = np.deg2rad(req.r)
	p = np.deg2rad(req.p)
	y = np.deg2rad(req.y)
	qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)

	qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)

	qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
	
	qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
	
	return 	qx,qy,qz,qw
def imu():
	rospy.init_node("imu", anonymous=True)
	args = rospy.myargv(argv = sys.argv)
	pub = rospy.Publisher('imu', Vectornav, queue_size=10)
	ser = rospy.Service('convert_to_quaternion', conversion, convert_service)
	rate = rospy.Rate(40)
	serial_port = rospy.get_param('/imu/port')  # reads the port name
	# serial_port = '/dev/ttyUSB0'
	serial_baud = 115200
	port = serial.Serial(serial_port,serial_baud,timeout=3.0) #serial input
	config = '$VNYMR,07,40*59'
	port.write = (config.encode()) 
	msg = Vectornav()     
	while not rospy.is_shutdown():
		ser_bytes = port.readline().decode('utf-8') #read imu_paras from port
		rospy.loginfo(ser_bytes)
		if 'VNYMR' in ser_bytes:
			imu_paras = ser_bytes.split(',')
			if len(imu_paras)<13:
				continue
			y = float(imu_paras[1].replace("\x00","").replace("\r\n",""))
			p = float(imu_paras[2].replace("\x00","").replace("\r\n",""))
			r= float(imu_paras[3].replace("\x00","").replace("\r\n",""))            
			x,y,z,w = convert_to_quaternion(r, p ,y)
			
			msg.header.frame_id="imu1_frame"
			msg.header.stamp.secs=rospy.Time.now().secs
			msg.header.stamp.nsecs=rospy.Time.now().nsecs

			msg.imu.orientation.x= float(x)
			msg.imu.orientation.y= float(y)
			msg.imu.orientation.z= float(z)
			msg.imu.orientation.w= float(w)
			msg.imu.angular_velocity.x= float(imu_paras[10].replace("\x00","").replace("\r\n",""))
			msg.imu.angular_velocity.y= float(imu_paras[11].replace("\x00","").replace("\r\n",""))
			msg.imu.angular_velocity.z= float(imu_paras[12][:10].replace("\x00","").replace("\r\n",""))
			msg.imu.linear_acceleration.x= float(imu_paras[7].replace("\x00","").replace("\r\n",""))
			msg.imu.linear_acceleration.y= float(imu_paras[8].replace("\x00","").replace("\r\n",""))
			msg.imu.linear_acceleration.z= float(imu_paras[9].replace("\x00","").replace("\r\n",""))
			
			msg.mag_field.magnetic_field.x = float(imu_paras[4].replace("\x00","").replace("\r\n",""))
			msg.mag_field.magnetic_field.y = float(imu_paras[5].replace("\x00","").replace("\r\n",""))
			msg.mag_field.magnetic_field.z = float(imu_paras[6].replace("\x00","").replace("\r\n",""))

			msg.raw_data = str(imu_paras)
			rospy.loginfo(msg)
			pub.publish(msg)
			rate.sleep()
	                     
	

if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        pass
