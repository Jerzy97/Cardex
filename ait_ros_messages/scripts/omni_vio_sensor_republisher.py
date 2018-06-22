#!/usr/bin/python

#   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name AIT nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# File: omni_vio_sensor_republisher.py
# Created on: 28.04.16
# Author: Christoph Tobler

from __future__ import print_function, division

__author__ = 'christoph'

import sys
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu
from ait_ros_messages.msg import VioSensorMsg
import time
import numpy as np
import genpy


uvc_node_name = "/uvc_camera"
init_0 = 0
init_1 = 0
init_2 = 0
init_3 = 0
init_4 = 0

def vio_sensor_cb0(data):
    global cam0_pub, cam1_pub, init_0
    if init_0 == 0:
        cam0_pub = rospy.Publisher(uvc_node_name + "/cam_0/image_raw", Image, queue_size=1)
        cam1_pub = rospy.Publisher(uvc_node_name + "/cam_1/image_raw", Image, queue_size=1)
        init_0 = 1

    cam0_pub.publish(data.left_image)
    cam1_pub.publish(data.right_image)

def vio_sensor_cb1(data):
    global cam2_pub, cam3_pub, init_1
    if init_1 == 0:
        cam2_pub = rospy.Publisher(uvc_node_name + "/cam_2/image_raw", Image, queue_size=1)
        cam3_pub = rospy.Publisher(uvc_node_name + "/cam_3/image_raw", Image, queue_size=1)
        init_1 = 1

    cam2_pub.publish(data.left_image)
    cam3_pub.publish(data.right_image)

def vio_sensor_cb2(data):
    global cam4_pub, cam5_pub, init_2
    if init_2 == 0:
        cam4_pub = rospy.Publisher(uvc_node_name + "/cam_4/image_raw", Image, queue_size=1)
        cam5_pub = rospy.Publisher(uvc_node_name + "/cam_5/image_raw", Image, queue_size=1)
        init_2 = 1

    cam4_pub.publish(data.left_image)
    cam5_pub.publish(data.right_image)

def vio_sensor_cb3(data):
    global cam6_pub, cam7_pub, init_3
    if init_3 == 0:
        cam6_pub = rospy.Publisher(uvc_node_name + "/cam_6/image_raw", Image, queue_size=1)
        cam7_pub = rospy.Publisher(uvc_node_name + "/cam_7/image_raw", Image, queue_size=1)
        init_3 = 1

    cam6_pub.publish(data.left_image)
    cam7_pub.publish(data.right_image)

def vio_sensor_cb4(data):
    global cam8_pub, cam9_pub, init_4
    if init_4 == 0:
        cam0_pub = rospy.Publisher(uvc_node_name + "/cam_8/image_raw", Image, queue_size=1)
        cam1_pub = rospy.Publisher(uvc_node_name + "/cam_9/image_raw", Image, queue_size=1)
        init_4 = 1

    cam8_pub.publish(data.left_image)
    cam9_pub.publish(data.right_image)


if __name__ == "__main__":

    rospy.init_node('image_publisher')

    rospy.Subscriber(uvc_node_name + "/vio_sensor_0", VioSensorMsg, vio_sensor_cb0, queue_size=1)
    rospy.Subscriber(uvc_node_name + "/vio_sensor_1", VioSensorMsg, vio_sensor_cb1, queue_size=1)
    rospy.Subscriber(uvc_node_name + "/vio_sensor_2", VioSensorMsg, vio_sensor_cb2, queue_size=1)
    rospy.Subscriber(uvc_node_name + "/vio_sensor_3", VioSensorMsg, vio_sensor_cb3, queue_size=1)
    rospy.Subscriber(uvc_node_name + "/vio_sensor_4", VioSensorMsg, vio_sensor_cb4, queue_size=1)

    rospy.spin()
