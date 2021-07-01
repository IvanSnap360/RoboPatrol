import rospy
import rospkg
from kml_parser.srv import execute_parse,execute_parseRequest,execute_parseResponse


class core():
    def __init__(self) -> None:
        self._kml_parser_service = rospy.ServiceProxy("/bot/system/KML_parser/read",execute_parse)

    def getKMLfile(self,file_name:str):
        self._kml_parser_service(file_name)

    def convertGPS2Local(self) -> None:
        pass

    def core(self):
        self.getKMLfile("route_1.kml")
        

