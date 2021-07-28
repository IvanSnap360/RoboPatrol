#! /usr/bin/env python 
"""
installing:
pip3 install sparkfun-ublox-gps
pip3 install spidev
"""
import time
import rospy
import serial
from std_msgs.msg import UInt32
from gps_common.msg import GPSFix
from geometry_msgs.msg import PoseStamped

rospy.init_node("gps_node")
gps_dir = rospy.get_param("/gps_dir","/gps")
gps_port = rospy.get_param("{}/port".format(gps_dir),"/dev/ttyTHS1")
gps_baud = rospy.get_param("{}/baudrate".format(gps_dir),9600)


port = serial.Serial(gps_port, gps_baud, timeout=2)

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
        line = str(port.readline())
        # print(line)
        line_array = line.split(',')
        # print(line_array)
        index = line_array[0]
        if (index == "$GNGGA"):
            sat = line_array[7]
            if (sat != ' '):
                sat = float(sat)
                if (sat >= 4):

                    lon = float(line_array[2])
                    lat = float(line_array[4])

                    lat = float("{:.5f}".format(lat))
                    lon = float("{:.5f}".format(lon))

                    geo_msg = GPSFix()
                    geo_msg.header.stamp = rospy.Time.now()
                    geo_msg.status.satellites_visible = sat
                    geo_msg.longitude = round(lon,6)
                    geo_msg.latitude = round(lat,6)
                    gps_pub.publish(geo_msg)

                    if (lon != 0 and lat != 0):
                        msg = PoseStamped()
                        msg.header.stamp = rospy.Time.now()
                        msg.pose.position.x = round(lat,6) 
                        msg.pose.position.y = round(lon,6)
                        pos_pub.publish(msg)
                else:
                    rospy.logwarn("wait coordinates... sattelite count {}".format(sat))
            else:
                rospy.logwarn("wait satelites...")

       
if __name__ == '__main__':
    run()
