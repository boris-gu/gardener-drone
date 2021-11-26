#!/usr/bin/env python3

# ========================
#  Распознавание маркеров
# ========================

import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

FONT = cv2.FONT_HERSHEY_PLAIN


def callback(data):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
        rospy.loginfo(e)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # Словарь
    arucoParam = aruco.DetectorParameters_create()  # Параметры
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, arucoDict,
                                                          parameters=arucoParam)
    if not(ids is None):
        frame = aruco.drawDetectedMarkers(frame, corners)
        ids = ids.astype(str)
        for id, cor in zip(ids, corners):
            # Расчет середины маркера
            locationText = cor[0][0] + (cor[0][2] - cor[0][0])/2
            # Преобразования, чтобы получить кортеж int
            locationText = tuple(locationText.astype(int).tolist())
            cv2.putText(frame, *id, locationText, FONT,
                        2, (255, 0, 200), 2)

    try:
        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
    except CvBridgeError as e:
        rospy.loginfo(e)


rospy.init_node('ArUco_Detected', anonymous=True)
image_sub = rospy.Subscriber('/gardener_drone/usb_cam/image_raw',
                             Image, callback, queue_size=20)
rospy.loginfo('Start Subscriber')
image_pub = rospy.Publisher('gardener_aruco_detected', Image, queue_size=20)
rospy.loginfo('Start Publisher')
rospy.spin()
