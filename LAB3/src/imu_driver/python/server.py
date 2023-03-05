from __future__ import print_function
from imu_driver.srv import conversion, conversionResponse
import numpy as np
import rospy



def convert_service(req):
    r = np.deg2rad(req.r)
    p = np.deg2rad(req.p)
    y = np.deg2rad(req.y)
    
    qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
    
    qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
    
    qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
    
    qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
        
    print("From euler angles roll:%s, pitch:%s and yaw:%s"%(req.r, req.p, req.y))
    print("We get quaternions as: \nqx:%s \nqy:%s \nqz:%s \nqw:%s"%(qx,qy,qz,qw))
	
    return 	conversionResponse(qx,qy,qz,qw)

def euler_to_quaternion_server():
    rospy.init_node("euler_to_quaternion_server")
    ser = rospy.Service('convert_to_quaternion', conversion, convert_service)
    print("Ready to convert")
    rospy.spin()


if __name__ == "__main__":
    euler_to_quaternion_server()


