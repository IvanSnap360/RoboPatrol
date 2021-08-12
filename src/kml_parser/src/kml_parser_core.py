#! /usr/bin/env python3
import rospy
from rospy.exceptions import ROSSerializationException
from sensor_msgs.msg import NavSatFix
import xml.etree.ElementTree as ET
import rospkg
from pykml import parser
from kml_parser.msg import point_marker,point_marker_array
from kml_parser.srv import execute_parse,execute_parseRequest,execute_parseResponse
rospack = rospkg.RosPack()

rospy.init_node("KML_parser")
ma = point_marker_array()

rate = rospy.Rate(2)

marker_pub = rospy.Publisher("/markers_array",point_marker_array,queue_size=10)
def parse(req:execute_parseRequest):
    global ma

    file = open("{}/KML_src/{}".format(rospack.get_path("kml_parser"),req.file_name),"r") 
    root = parser.parse(file).getroot()

    try:
        pms = root.Document.Folder.Placemark
    except:
        pms = root.Document.Placemark

    for p in pms:
        # print("point name:",p.name,"   [",p.Point.coordinates,"]")
        data = str(p.Point.coordinates).split(",")
        name = str(p.name)
        longitude   = float(data[0])
        latitude    = float(data[1])
        altitude    = float(data[2])
        
        marker = point_marker()
        marker.name = name
        marker.latitude = latitude
        marker.longtitude = longitude
        ma.markers.append(marker)
        
        log_out_msg = "\033[36m mark name\033[0m {} \n\t longitude: {} \n\t latitude: {} \n\t altitude: {} \n".format(name,longitude,latitude,altitude)
        # rospy.loginfo(log_out_msg)
        print(log_out_msg)
        

rospy.Service("/kml_parser",execute_parse,parse)
while not rospy.is_shutdown():
    marker_pub.publish(ma)
    rate.sleep()