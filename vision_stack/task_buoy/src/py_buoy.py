#! /usr/bin/env python

import sys
import roslib
roslib.load_manifest('task_buoy')
import rospy
import actionlib
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from resources import topicHeader
from resources import tools

from ip_msgs.msg import buoyAction as actionMessagebuoyAction
from ip_msgs.msg import buoyFeedback
from ip_msgs.msg import buoyResult
from ip_msgs.msg import buoyGoal

from sensor_msgs.msg import Image

class buoyServer(object):


	def __init__(self, name):

		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, actionMessagebuoyAction, execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

		self._image = np.zeros((100,100,3), np.uint8)
		self.kernel = np.ones((3,3),np.uint8)
		self.bridge = CvBridge()
		self.pub = rospy.Publisher(topicHeader.CAMERA_FRONT_BUOY_IMAGE, Image, queue_size=20)
		self.sub = rospy.Subscriber(topicHeader.CAMERA_FRONT_RAW_IMAGE, Image, self.image_cb)

		self.allVals = [[[0 for i in range(256)] for j in range(256)] for k in range(256)]
		with open(sys.argv[1]) as f:
			for i in range(0, 256):
				for j in range(0, 256):
					for k in range(0, 256):
						x = f.read(1)
						if x is not None:
							self.allVals[i][j][k] = x

		rospy.loginfo('Server has started')

	def image_cb(self, data):
		try:
			numpy.copyto(self._image, self.bridge.imgmsg_to_cv2(data, "bgr8"))
		except CvBridgeError as e:
			print(e)

	def execute_cb(self, goal):
		_detected = False



		if self._as.is_preempt_requested():
			rospy.loginfo('%s: Preempted' % self._action_name)
			self._as.set_preempted()
			success = False
			return

		self.detect_buoy()
		ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
		pub.publish(ros_image)


	def detect_buoy(self):
		if not self._image is None:
			cv2.imshow('Input', self._image)

			for i in range(0, self._image.width):
				for j in range(0, self._image.height):
					pixel_value = cv.Get2D(self._image, i, j)
					k = self.allVals[pixel_value[2]][pixel_value[1]][pixel_value[0]]
					if k==0:
						self._image.itemset((i, j, 0), 255)
						self._image.itemset((i, j, 1), 0)
						self._image.itemset((i, j, 0), 0)
					elif k==1:
						self._image.itemset((i, j, 0), 255)
						self._image.itemset((i, j, 1), 255)
						self._image.itemset((i, j, 0), 0)
					elif k==2:
						self._image.itemset((i, j, 0), 0)
						self._image.itemset((i, j, 1), 0)
						self._image.itemset((i, j, 0), 0)

			_imageBW = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
			cv2.imshow("BW Image", _imageBW)
			_imageBW = cv2medianBlur(_imageBW, 5);
			_imageBW = cv2.erode(_imageBW, kernel, iterations = 1)
			cv2.imshow("To process", _imageBW)
			

			circles = cv2.HoughCircles(_imageBW,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)

			circles = np.uint16(np.around(circles))
			for i in circles[0,:]:
				# draw the outer circle
				cv2.circle(_imageBW,(i[0],i[1]),i[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(_imageBW,(i[0],i[1]),2,(0,0,255),3)

			cv2.imshow("Output", _imageBW)

			self._image = _imageBW

			if(len(circles)>1):
				return True
			else:
				return False
		
if __name__ == '__main__':
	rospy.init_node('buoy_server_nn', log_level=(rospy.DEBUG if tools.getVerboseTag(sys.argv) else rospy.INFO))
	buoyServer('buoy_nn')
	rospy.spin()