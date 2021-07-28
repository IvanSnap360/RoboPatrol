#! /usr/bin/env python3
"""
installing:
pip3 install sparkfun-ublox-gps
pip3 install spidev
"""
import time
import rospy
import serial
from ublox_gps import UbloxGps
from std_msgs.msg import UInt32
from gps_common.msg import GPSFix
from geometry_msgs.msg import PoseStamped

rospy.init_node("gps_node")
gps_dir = rospy.get_param("/gps_dir","/gps")
gps_port = rospy.get_param("{}/port".format(gps_dir),"/dev/ttyTHS1")
gps_baud = rospy.get_param("{}/baudrate".format(gps_dir),9600)


port = serial.Serial(gps_port, gps_baud, timeout=2)
gps = UbloxGps(port)

gps_pub = rospy.Publisher("{}/gps_data".format(gps_dir),GPSFix,queue_size=10)
pos_pub = rospy.Publisher("/local_xy_origin",PoseStamped,queue_size=20)
# sat_pub = rospy.Publisher("{}/sattelites".format(gps_dir),UInt32,queue_size=10)
rospy.loginfo("Start gps node")

lat = 0.0
lon = 0.0


def run():
    global lat
    global lon
    last_time = 0

    while not rospy.is_shutdown():

        geo = gps.geo_coords()
        lat = geo.lat
        lon = geo.lon
        

        geo_msg = GPSFix()
        geo_msg.header.stamp = rospy.Time.now()
        geo_msg.longitude = round(lon,6)
        geo_msg.latitude = round(lat,6)
        gps_pub.publish(geo_msg)

        if (lon != 0 and lat != 0):
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = round(lat,6) 
            msg.pose.position.y = round(lon,6)
            pos_pub.publish(msg)
    
       
if __name__ == '__main__':
    run()
