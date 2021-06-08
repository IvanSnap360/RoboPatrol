import rospy
from rospy.exceptions import ROSSerializationException
from sensor_msgs.msg import NavSatFix
import xml.etree.ElementTree as ET
import rospkg
from pykml import parser
from kml_parser.srv import execute_parse,execute_parseRequest,execute_parseResponse
rospack = rospkg.RosPack()

rospy.init_node("KML_parser")

def parse(req:execute_parseRequest):

    file = open("{}/KML_src/{}".format(rospack.get_path("kml_parser"),req.file_name),"r") 
    root = parser.parse(file).getroot()

    try:
        pms = root.Document.Folder.Placemark
    except:
        pms = root.Document.Placemark

    for p in pms:
        # print("point name:",p.name,"   [",p.Point.coordinates,"]")
        data = str(p.Point.coordinates).split(",")
        longitude   = float(data[0])
        latitude    = float(data[1])
        altitude    = float(data[2])
        
        log_out_msg = "\033[36m mark name\033[0m {} \n\t longitude: {} \n\t latitude: {} \n\t altitude: {} \n".format(p.name,longitude,latitude,altitude)
        # rospy.loginfo(log_out_msg)
        print(log_out_msg)

