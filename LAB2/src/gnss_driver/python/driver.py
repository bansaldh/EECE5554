#!/usr/bin/env python3

import rospy
import serial
import utm
import roslib
import sys
import math
roslib.load_manifest('gnss_driver')
from gnss_driver.msg import gnss_msg

def gnss_driver():
	args = rospy.myargv(argv = sys.argv)
	gps_pub = rospy.Publisher('gnss', gnss_msg, queue_size=10)
	msg=gnss_msg()
	rospy.init_node("gnss_driver", anonymous=True)
	rate = rospy.Rate(1)
	serial_port = rospy.get_param('port', args[1])
	# serial_port = rospy.get_param('port', '/dev/pts/2')
	serial_baud = 57600
	port = serial.Serial(serial_port,serial_baud,timeout=3.0)

	f=open("Name_of_file.txt",'w') #change the file name from the data file in hand
	while not rospy.is_shutdown():
		line = port.readline()
		line = str(line, 'UTF-8')
		# if line.startswith('$GNGGA'): # this command not working properly.
		if 'GNGGA' in line:
			try :
				navigation_ = line[6:].split(',')
			except:
				rospy.loginfo("Exception data: "+line)
				continue

				
				# time handling 
			time_utc = float(navigation_[1]) #originally in hhmmss.ss
			hours = time_utc//10000
			minutes = int((time_utc%10000)/100)
			seconds = math.ceil((time_utc%100)*100)
			nanoseconds = math.ceil((time_utc%1)*100)

			# conversion of lat/lon to decimal

			if navigation_[3]=='S':
				latitude = -1*int(float(navigation_[2][:2]) + float(navigation_[2][2:]))/60
			else:
				latitude = int(float(navigation_[2][:2]) + float(navigation_[2][2:]))/60

			if navigation_[5]=='W':
				longitude = -1*int(float(navigation_[4][:3]) + float(navigation_[4][3:]))/60
			else:
				longitude = int(float(navigation_[4][:3]) + float(navigation_[4][3:]))/60
			
			UTM_coordinate  = utm.from_latlon(latitude, longitude)

			quality_indicator=navigation_[6]


			if navigation_[10] == 'M': #if it is already in meters
				if navigation_[0] == '':
					altitude = 0.0
				else:
					altitude = float(navigation_[9])
			else:
				if navigation_[0] == '':
					altitude = 0.0
				else:
					altitude = float(navigation_[9]/100)

			HDOP_value = navigation_[8]
			frame_id = "GPS1_frame"
			# conversion of lat/lon to UTM
			s_UTM="%s,%s,%s,%s,%s,%s,%s,%s\n"%(frame_id,time_utc,UTM_coordinate[0],UTM_coordinate[1],UTM_coordinate[2],UTM_coordinate[3],navigation_[9],navigation_[8])
			f.write(s_UTM)	
			
			msg.Header.frame_id="GPS1_frame"
			msg.Header.stamp = rospy.Time.from_sec(rospy.Time(seconds, nanoseconds).to_sec())
			msg.Latitude=latitude
			msg.Longitude=longitude
			msg.Altitude=altitude
			msg.UTM_easting=float(UTM_coordinate[0])
			msg.UTM_northing=float(UTM_coordinate[1])
			msg.Zone=int(UTM_coordinate[2])
			msg.Letter="%s"%(UTM_coordinate[3])
			msg.HDOP = float(HDOP_value)
			msg.gps_quality=int(quality_indicator)
			msg.UTC=float(time_utc)
			# rospy.loginfo(msg)
			gps_pub.publish(msg)
			rate.sleep()
		else:
			rospy.loginfo("Not appropriate data")
				
	f.close()
	
	

if __name__ == '__main__':
    try:
        gnss_driver()
    except rospy.ROSInterruptException:
        pass

