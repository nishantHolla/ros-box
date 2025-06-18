#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException
from rectangle_area_service.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse

def handle_rectangle_area(req):
	rospy.loginfo(req)
	return RectangleAreaServiceResponse(req.width * req.height)

def node():
	rospy.init_node("ras_server", anonymous=False)

	service = rospy.Service("rectangle_area", RectangleAreaService, handle_rectangle_area)
	print("Started service /rectangle_area")

	rospy.spin()

if __name__ == '__main__':
	try:
		node()
	except ROSInterruptException:
		pass
