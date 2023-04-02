import rospy
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from time import sleep

# Image Detection Global Variables
bridge = CvBridge()
stop_sign_detector = cv2.CascadeClassifier('haar/stop_sign_classifier_2.xml')

# Control Global Variables
has_stopped = False
speed = 30

# Detecta si hay una señal de stop en la image
def check_stop_signal(image):
	global stop_sign_detector

	# Analizamos la imagen con el modelo
	stop_sign = stop_sign_detector.detectMultiScale(image, 1.3, 5, minSize = (100, 100))
	print(stop_sign)
	# Si el modelo regresa alguna señal se regresa True, caso
	# contrario false
	if len(stop_sign) > 0:
		return True
	else:
		return False

def pub_speed(any_signal):
	global speed_pub
	global has_stopped
	print(has_stopped)
	if any_signal:
		if has_stopped:
			speed_pub.publish(speed)
		else:
			speed_pub.publish(0)
			print("Inician 3 seg")
			sleep(9)
			print("Terminan 3 seg")
			has_stopped = True
	else:
		speed_pub.publish(speed)
		has_stopped = False


def process_img(image):
	return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

# Funcion de callback para el suscriber de la camara
def callimg(data):
	cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')

	procesed_img = process_img(cv_image)

	any_signal = check_stop_signal(procesed_img)

	pub_speed(any_signal)
	cv2.imshow("Signal Detection", procesed_img)	
	cv2.waitKey(3)

def main():
	global speed_pub

	# Publisher para mandar la velocidad al topic /speed
	speed_pub = rospy.Publisher("/speed", Float64, queue_size = 10)
	# Suscriber para recibir las imagenes de el topic /camera/rgb/raw
	cam_sub = rospy.Subscriber('/camera/rgb/raw', Image, callimg)

	rospy.init_node('signal_detector')

	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
