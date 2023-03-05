from __future__ import print_function
import sys
import rospy
from imu_driver.srv import *

def euler_to_quaternion_client(r, p, y):
    rospy.wait_for_service('convert_to_quaternion')
    try:
        convert_to_quaternion = rospy.ServiceProxy('convert_to_quaternion', conversion)
        resp1 = convert_to_quaternion(r, p, y)
        return [resp1.qx, resp1.qy, resp1.qz, resp1.qw]
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

def usage():
    return "%s [r p y]"%sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 4:
        r = float(sys.argv[1])
        p = float(sys.argv[2])
        y = float(sys.argv[3])

    else:
        print(usage)
        sys.exit()
    
    print("Requesting roll:%s, pitch:%s and yaw:%s"%(r, p, y))
    
    print("From eulers roll:%s, pitch:%s and yaw:%s \nwe get the following quaternions:%s"%(r, p, y, euler_to_quaternion_client(r, p, y)))
