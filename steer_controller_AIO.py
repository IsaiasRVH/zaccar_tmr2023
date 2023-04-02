#Split-screen
import rospy
import cv2 as cv
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

steer_angle = None
crop_img_height = 450


###########################################################
# Calcula la posición del volante donde:
# data[0] es izq_x1
# data[1] es izq_y1
# data[2] es izq_x2
# data[3] es izq_y2
# data[4] es der_x1
# data[5] es der_y1
# data[6] es der_x2
# data[7] es der_y2
#
# Para probar los pesos simplemente reemplaza la formula
###########################################################
def calc_steering(data):
	steer_raw = 0.02142 - 0.0002624 * data[0] - 0.000007799 * data[2] + 0.0005376 * data[3] + 0.000001584 * data[4] - 0.000002633 * data[6] - 0.00006334 * data[7]
	return steer_raw

def get_aoi(image):
	mask = np.zeros(image.shape, dtype = "uint8")
	roi_corners = np.array([[(140,940), (500, 520), (925,520), (1279,760), (1279,940)]], dtype=np.int32)

	ignore_mask_color = (255,)
	cv.fillPoly(mask, roi_corners, ignore_mask_color)
	# from Masterfool: use cv.fillConvexPoly if you know it's convex

	# apply the mask
	masked_image = cv.bitwise_and(image, mask)
	return masked_image

def proccess_img(image):
	gray_img = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
	masked_image = get_aoi(gray_img)
	#cv.imshow("Masked Image", masked_image)
	gaus = cv.blur(masked_image, (5,5))
	r,bins = cv.threshold(gaus, 115, 255, cv.THRESH_BINARY)
	crop = bins[-crop_img_height:,:]
	edges = cv.Canny(crop, 50, 150)
	return edges

def append_zeros(data, n):
	for i in range(n):
		data.append(0)
	return data

def get_points(image):
	linesP = cv.HoughLinesP(image, rho=1, theta = np.pi/180, threshold = 20, minLineLength = 110, maxLineGap = 400)
	points = []
	if linesP is not None:
		m_izq = []
		m_der = []
		b_izq = []
		b_der = []
		half_width = int(len(image[0])/2)
		
		for i in range(0, len(linesP)):
			l = linesP[i][0]
			m = (l[3] - l[1]) / (l[2] - l[0])
			b = l[1] - m * l[0]
			print("M: ", m)
			if m > 0.04 and l[0] >= half_width:
				m_der.append(m)
				b_der.append(b)
			elif m < -0.04 and l[0] <= half_width:
				m_izq.append(m)
				b_izq.append(b)
		if len(m_der) != 0:
			m_der_mean = np.mean(m_der)
			b_der_mean = np.mean(b_der)
			pointC = (int((0-b_der_mean)/m_der_mean), 0)
			pointD = (int((len(image[:,1])-b_der_mean)/m_der_mean), len(image[:,1]))
			cv.line(image, pointC , pointD, (255,0,0), 3, cv.LINE_AA)
			points.append(pointC[0])
			points.append(pointC[1])
			points.append(pointD[0])
			points.append(pointD[1])
		else:
			points = append_zeros(points, 4)

		if len(m_izq) != 0:
			m_izq_mean = np.mean(m_izq)
			b_izq_mean = np.mean(b_izq)
			pointA = (int((0-b_izq_mean)/m_izq_mean), 0)
			pointB = (int((len(image[:,1]) - b_izq_mean)/m_izq_mean), len(image[:,1]))
			cv.line(image, pointA, pointB, (255,0,0), 3, cv.LINE_AA)
			points.append(pointA[0])
			points.append(pointA[1])
			points.append(pointB[0])
			points.append(pointB[1])
		else:
			points = append_zeros(points, 4)
	else:
		points = append_zeros(points, 8)
	
	return points

def callimg(data):
	global steer_angle
	global steer_pub
	
	cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
	edges = proccess_img(cv_image)
	points = get_points(edges)
	
	#cv.imshow('Crop', edges)
	steer_angle = calc_steering(points)
	print("Steer:", steer_angle, "Points:", points)
	steer_pub.publish(steer_angle)
	cv.waitKey(3)

def main():
	global steer_angle
	global steer_pub
	
	steer_pub = rospy.Publisher('/steering', Float64, queue_size = 10)

	sub = rospy.Subscriber('/camera/rgb/raw', Image, callimg)
	
	rospy.init_node('imagen')
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv.destroyAllWindows()
		
if __name__ == '__main__':
	main()
