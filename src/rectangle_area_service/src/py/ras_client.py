#!/usr/bin/env python
import rospy
from rospy.exceptions import ROSInterruptException
from rectangle_area_service.srv import RectangleAreaService, RectangleAreaServiceRequest, RectangleAreaServiceResponse

def send_request(a, b):
	try:
		client = rospy.ServiceProxy("rectangle_area", RectangleAreaService)
		response = client(a, b)
		return response.area

	except rospy.ServiceException(e):
		print(f'Server error: {e}')

def node():
	rospy.init_node("ras_client", anonymous=False)

	rospy.wait_for_service("rectangle_area")

	while not rospy.is_shutdown():
		try:
			a = int(input("width: "))
			b = int(input("height: "))
		except:
			break

		sum = send_request(a, b)
		print(f'area: {sum}')


# Start the node if the file is executed
if __name__ == "__main__":
	try:
		node()
	except ROSInterruptException:
		pass
