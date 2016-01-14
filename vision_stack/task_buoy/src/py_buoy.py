#! /usr/bin/env python

import sys
import roslib
roslib.load_manifest('task_buoy')
import rospy
import actionlib
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from resources import topicHeader
from resources import tools

import actionmsg.msg
from ip_msgs.msg import buoyAction as actionMessagebuoyAction
from ip_msgs.msg import buoyFeedback
from ip_msgs.msg import buoyResult
from ip_msgs.msg import buoyGoal

from sensor_msgs.msg import Image

class buoyServer(object):

	_feedback = actionmsg.msg.buoyFeedback()
	_result   = actionmsg.msg.buoyResult()

	def __init__(self, name, filepath, sampling_rate=10):

		self.rate = rospy.Rate(10)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, actionMessagebuoyAction, execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

		self._image = np.zeros((480,640,3), np.uint8)
                self._imageBW = np.zeros((480, 640, 3), np.uint8)
		self.kernel = np.ones((3,3),np.uint8)
		self.bridge = CvBridge()
    
                self.counter = -1
                self.sampling_rate = sampling_rate

		self.pub = rospy.Publisher(topicHeader.CAMERA_FRONT_BUOY_IMAGE, Image, queue_size=20)
		self.sub = rospy.Subscriber(topicHeader.CAMERA_FRONT_RAW_IMAGE, Image, self.image_cb)

		self.allVals = [[[0 for i in range(256)] for j in range(256)] for k in range(256)]
		with open(filepath) as f:
			for i in range(0, 256):
				for j in range(0, 256):
					for k in range(0, 256):
						x = f.read(1)
						if x is not None:
							self.allVals[i][j][k] = x

		rospy.loginfo('Server has started')

	def image_cb(self, data):
		try:
			self._image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                        if self._image.shape[2] != 3:
                            print "3 channels not present in this image!!!"
		except CvBridgeError as e:
			print(e)

	def execute_cb(self, goal):
		_detected = False

		print "goal - "
		print goal

		while 1:
			
			#if self._as.is_preempt_requested():
				#rospy.loginfo('%s: Preempted' % self._action_name)
				#self._as.set_preempted()
				#success = False
				#return

			detected = self.detect_buoy()
			ros_image = self.bridge.cv2_to_imgmsg(self._image, encoding="passthrough")
			self.pub.publish(ros_image)

			if(detected):
				self._result.sequence.append(2)

			self.rate.sleep()

        def detect_buoy(self):
                #self.counter += 1
		#if not self._image is None and self.counter % self.sampling_rate == 0:
		if not self._image is None:
			cv2.imshow('Input', self._image)
			
			# (height,width,channels) = self._image.shape
                        print "Shape: ", self._image.shape
                        if len(self._image.shape) >= 3:
                            width = self._image.shape[0]
                            height = self._image.shape[1]
                        else:
                            width = 480
                            height = 600

			for i in range(0, width):
				for j in range(0, height):
					k = self.allVals[self._image[i][j][2]][self._image[i][j][1]][self._image[i][j][0]]
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
			#cv2.imshow("BW Image", _imageBW)
			_imageBW = cv2.medianBlur(_imageBW, 5);
			_imageBW = cv2.erode(_imageBW, self.kernel, iterations = 1)
			#cv2.imshow("To process", _imageBW)
			

			circles = cv2.HoughCircles(_imageBW,cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
                        
                        ## Testing Block
                        ## Remove if and when moving into master branch
                        if circles == None:

                            rospy.logerr("Hough Circles did not identify any circle")
                            #sys.exit(1)

			if not circles is None:
				#circles = np.uint16(np.around(circles))
				for i in circles[0,:]:
					# draw the outer circle
					cv2.circle(_imageBW,(i[0],i[1]),i[2],(0,255,0),2)
					# draw the center of the circle
					cv2.circle(_imageBW,(i[0],i[1]),2,(0,0,255),3)

			cv2.imshow("Output", _imageBW)

			self._imageBW = _imageBW

			if circles == None:
                                rospy.loginfo("No circles found!")
				return False
			else:
                                rospy.loginfo("some circles were found!")
				return True
		
if __name__ == '__main__':
	rospy.init_node('buoy_server_nn', log_level=(rospy.DEBUG if tools.getVerboseTag(sys.argv) else rospy.INFO))
        if len(sys.argv) <= 1:

            rospy.logerr("You need to provide the path to the array file as the first CLI argument")
            sys.exit(1)

	buoyServer('buoy_nn', sys.argv[1])
	rospy.spin()
