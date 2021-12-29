#!/usr/bin/env python3

# ====================================
#  Определение расстояния до маркеров
# ====================================

import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aruco_calibration import Calibration as clb

FONT = cv2.FONT_HERSHEY_DUPLEX


def callback(data):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
        rospy.loginfo(e)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters,
                                                          cameraMatrix=camera_matrix,
                                                          distCoeff=dist_coef)
    if np.all(ids is not None):
        # TODO: ПОЧИНИТЬ (возможно только калибровка)
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.2, camera_matrix,
                                                                   dist_coef)
        for i in range(0, len(ids)):
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix,
                           dist_coef, rvec[i], tvec[i], 0.2)
        cv2.putText(frame, 'distance (m)', (20, 30), FONT,
                    1, (255, 0, 60), 2, cv2.LINE_AA)
        distance = np.sum(tvec ** 2, axis=2) ** 0.5
        for i in range(len(distance)):
            cv2.putText(frame, str(distance[i])[1:-1] + ' id' + str(ids[i])[1:-1],
                        (20, 70+40*i), FONT,
                        1, (255, 0, 60), 2, cv2.LINE_AA)

    try:
        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
    except CvBridgeError as e:
        rospy.loginfo(e)


# calibration_save.yaml - уже проведена калибровка
camera_matrix, dist_coef = clb.loadCoefficients('calibration_save.yaml')
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

rospy.init_node('ArUco_Detected', anonymous=True)
image_sub = rospy.Subscriber('/realsense/color/image_raw',
                             Image, callback, queue_size=20)
rospy.loginfo('Start Subscriber')
image_pub = rospy.Publisher(
    'realsense/aruco/location_img', Image, queue_size=20)
rospy.loginfo('Start Publisher')
rospy.spin()
