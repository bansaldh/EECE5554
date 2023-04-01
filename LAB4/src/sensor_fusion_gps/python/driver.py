#!/usr/bin/env python3

import rospy
import serial
import utm
import roslib
roslib.load_manifest('gps_driver')
from gps_driver.msg import gps_msg
serial_port1 = rospy.get_param('/gps_driver/port')


def gps_driver():
	
	gps_publ = rospy.Publisher('gps', gps_msg, queue_size=10)
	message1=gps_msg()
	rospy.init_node("gps_driver", anonymous=True)
	rate = rospy.Rate(1)        #The rate is determined by user. In our case, since we were taking data for approximately 5 minutes, 					      we set the rate at 1Hz
	
	#The following line can be used if launch file is not to be used.
	#serial_port ='/dev/pts/2' #Enter the serial port for which the GPS puck is connected.
	#If Launch were to be used.
	serial_port = serial_port1
	
	serial_baud = 4800          #Ensure that the baud rate is set to 4800
	port = serial.Serial(serial_port,serial_baud,timeout=3.0)
	
	while(not rospy.is_shutdown()):
		linesep1 = str(port.readline())  # To read each line from the GPS puck
		data1 = linesep1.split("b'")      # Each Line starts with a b and hence to split it and remove it
		data2 = data1[1]                 # We want the remaining portion of the string as that contains the GPS data
		#rospy.loginfo(data2)
		data3 = data2.split(",")         # Splitting the data by commas
		if "$GPGGA" in data3[0]:
			#The following if condition is to ensure we dont get empty strings.
			#The first string of the Line is the time in UTC. { Zeroth String is the GPGGA }
			time1 = float(data3[1])  
			#Converting obtained time to seconds.
			seconds1 = (time1//10000)*3600 + ((time1%10000)//100 )* 60 + ((time1%10000)%100 ) 
			time2 = rospy.Time.from_sec(seconds1)
			
			#The latitude is the Second String in the Line. Before converting to float, while it is in the string      			 state itself, the hours and minutes are separately extrated and then converted.
			latitude1=float(data3[2][:2])+float(data3[2][2:])/60.0
			latitude_direction=data3[3]
			if latitude_direction=='S':
				latitude1=-latitude1
			
			#The longitude is the Fourth String in the Line. Before converting to float, while it is in the string state 				 itself, the hours and minutes are separately extrated and then converted.
			longitude1=float(data3[4][:3])+float(data3[4][3:])/60.0
			longitude_direction=data3[5]
			if longitude_direction=='W':
				longitude1=-longitude1
			
			q_check=data3[6]                        #Quality Indicator
			sat_num=data3[7]                        #No. of Satellites
			HDOP_data = data3[8]                    #Horizontal dilution of Precision
			Altitude1=float(data3[9])               #Altitude in Meters
			
			frame_id = "GPS1_frame"                #Frame Id is fixed as it is taken from the same GPS Puck
			UTM_C=utm.from_latlon(latitude1,longitude1)	
			#The UTM Coordinate output is of the form [UTM_easting, UTM_northing UTM Zone UTM Letter]
			
			message1.Header.frame_id="GPS1_FRAME"
			message1.Header.stamp=rospy.Time.now()
			message1.Header.seq+=1
			message1.Latitude=latitude1
			message1.Longitude=longitude1
			message1.Altitude=Altitude1
			message1.UTM_easting=float(UTM_C[0])
			message1.UTM_northing=float(UTM_C[1])
			message1.Zone=int(UTM_C[2])
			message1.Letter="%s"%(UTM_C[3])
			message1.HDOP = float(data3[8])
			message1.UTC = float(data3[1])
			#rospy.loginfo(message1)              # To print the message in the output screen. 
			gps_publ.publish(message1)
			rate.sleep()
			
				
	
if __name__ == '__main__':
    try:    	
    	gps_driver()
    except rospy.ROSInterruptException:
        pass
