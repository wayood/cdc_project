#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2