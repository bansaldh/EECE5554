#!/usr/bin/env python3

import rospy
import serial
import utm
import roslib
roslib.load_manifest('gps_driver')
from gps_driver.msg import gps_msg
sp = rospy.get_param('/gps_driver/s_port')

def gps_driver():

	gps_pub = rospy.Publisher('gps_message', gps_msg, queue_size=10)
	msg=gps_msg()
	rospy.init_node("gps_driver", anonymous=True)
	rate = rospy.Rate(1)
	serial_port =sp # amke sure the serial port number is correct
	serial_baud = 4800
	port = serial.Serial(serial_port,serial_baud,timeout=3.0)
	i=0
	f=open("Name_of_file.txt",'w') #change the file name from the data file in hand
	while(not rospy.is_shutdown()):
		line = port.readline().decode('utf8','ignore')
		if line[1:6] == 'GPGGA': #GPGGA string parsing
			data=line[6:].split(',')
			if data[5]!='0':
				# conversion of lat/lon to decimal
				time1 = data[1]
				lat=float(data[2][:2])+float(data[2][2:])/60.0
				lat_dir=data[3]
				if lat_dir=='S':
					lat=-lat
				lon=float(data[4][:3])+float(data[4][3:])/60.0
				lon_dir=data[5]
				if lon_dir=='W':
					lon=-lon
				status=data[6]
				sat_num=data[7] #No. of Satellites
				if data[9] =='':
					alt = 0.0
				else:
					alt=float(data[9]) #Altitude i Meters
				hdop = data[8]
				frame_id = "GPS1_frame"
				# conversion of lat/lon to UTM
				UTM_coordinate=utm.from_latlon(lat,lon)
				s_UTM="%s,%s,%s,%s,%s,%s,%s,%s\n"%(frame_id,time1,UTM_coordinate[0],UTM_coordinate[1],UTM_coordinate[2],UTM_coordinate[3],data[9],data[8])
				rospy.loginfo(msg)
				f.write(s_UTM)	
				#time handling
				time1 = float(data[1])
				seconds1 = (time1//10000)*3600 + ((time1%10000)//100 )* 60 + ((time1%10000)%100 )
				time2 = rospy.Time.from_sec(seconds1)
				
				msg.Header.frame_id="GPS1_frame"
				msg.Header.stamp=time2
				msg.Latitude=lat
				msg.Longitude=lon
				msg.Altitude=alt
				msg.UTM_easting=float(UTM_coordinate[0])
				msg.UTM_northing=float(UTM_coordinate[1])
				msg.Zone=int(UTM_coordinate[2])
				msg.Letter="%s"%(UTM_coordinate[3])
				msg.HDOP = float(data[8])
				rospy.loginfo(msg)
				gps_pub.publish(msg)
				rate.sleep()
				
	f.close()
	
	

if __name__ == '__main__':
    try:
        gps_driver()
    except rospy.ROSInterruptException:
        pass
