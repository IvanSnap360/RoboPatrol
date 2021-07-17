#! /usr/bin/env python3 
"""
installing:
pip3 install sparkfun-ublox-gps
pip3 install spidev
"""
import rospy
import serial
from ublox_gps import UbloxGps

from gps_common.msg import GPSFix
from std_msgs.msg import UInt32

rospy.init_node("gps_node")
if rospy.has_param("/gps_dir"):
    gps_dir = rospy.get_param("/gps_dir","/gps")
    gps_port = rospy.get_param("{}/port".format(gps_dir),"/dev/ttyUSB0")
    gps_baud = rospy.get_param("{}/baudrate".format(gps_dir),9600)


port = serial.Serial(gps_port, gps_baud, timeout=2)
gps = UbloxGps(port)

gps_pub = rospy.Publisher("{}/gps_data".format(gps_dir),GPSFix,queue_size=10)
# sat_pub = rospy.Publisher("{}/sattelites".format(gps_dir),UInt32,queue_size=10)
rospy.loginfo("Start gps node")


def run():
    
    try:
        while not rospy.is_shutdown():
            try:
                geo = gps.geo_coords()
                geo_msg = GPSFix()
                geo_msg.longitude = round(geo.lon,6)
                geo_msg.latitude = round(geo.lat,6)
                gps_pub.publish(geo_msg)
                
            except:
                rospy.logwarn("GPS warning!!!")
                pass        
    finally:
        port.close()


if __name__ == '__main__':
    run()
