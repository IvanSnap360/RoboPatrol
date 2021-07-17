#! /usr/bin/env python3 
from yaml.error import Mark
import rospy
from rospy.exceptions import ROSSerializationException
from sensor_msgs.msg import NavSatFix
import xml.etree.ElementTree as ET
import rospkg
from pykml import parser
from kml_parser.srv import execute_parse,execute_parseRequest,execute_parseResponse
from visualization_msgs.msg import MarkerArray,Marker
rospack = rospkg.RosPack()

rospy.init_node("KML_parser")

marker_publisher = rospy.Publisher("/markers_array",MarkerArray,queue_size=10)
marray = MarkerArray()
def parse(req):

    file = open("{}/KML_src/{}".format(rospack.get_path("kml_parser"),req.file_name),"r") 
    root = parser.parse(file).getroot()

    try:
        pms = root.Document.Folder.Placemark
    except:
        pms = root.Document.Placemark
    
    count = 0
    marray.markers.clear()
    for p in pms:
        m = Marker()
        # print("point name:",p.name,"   [",p.Point.coordinates,"]")
        data = str(p.Point.coordinates).split(",")
        longitude   = float(data[0])
        latitude    = float(data[1])
        altitude    = float(data[2])
        
        log_out_msg = "\033[36m mark_name\033[0m {} \n\t longitude: {} \n\t latitude: {} \n\t altitude: {} \n".format(p.name,longitude,latitude,altitude)
        # rospy.loginfo(log_out_msg)
        # rospy.set_param("/route/points/{}/coordinates(lon,lan,alt)".format(p.name),[longitude,latitude,altitude])
        print(log_out_msg)
        
        m.header.frame_id = "/map"
        m.action = m.ADD
        m.type = m.SPHERE
        m.id = count
        m.text = str(count)
        m.color.r = 255
        m.color.g = 0
        m.color.b = 0
        m.pose.position.x = longitude
        m.pose.position.y = latitude
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        marray.markers.append(m)

        count += 1

    rospy.set_param("/route/points/count",count)
    res = execute_parseResponse()
    res.message =  "DONE"
    res.result = True
    return res

rospy.Service("/bot/system/KML_parser/read",execute_parse,parse)
while not rospy.is_shutdown():
    marker_publisher.publish(marray)