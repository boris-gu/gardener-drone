#!/usr/bin/env python3

# =====================
#  Зависнуть у маркера
# =====================

import numpy as np
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aruco_calibration import Calibration as clb
from drone_api import *
from math import pi, sqrt, sin, cos, atan2


FONT = cv2.FONT_HERSHEY_PLAIN


def toFixed(numObj, digits=0):
    return f'{numObj:.{digits}f}'


def callback(data):
    # Мост для преобразования изображения из формата ROS в формат OpenCV
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    except Exception as e:
        rospy.loginfo(e)
        return
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 6
    corners, ids, _ = aruco.detectMarkers(gray, aruco_6dict,
                                          parameters=parameters,
                                          cameraMatrix=camera_matrix,
                                          distCoeff=dist_coef)
    aruco.drawDetectedMarkers(frame, corners)
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, aruco_6size, camera_matrix,
                                                    dist_coef)
    global marker_pose
    if np.all(ids is not None):
        for i in range(len(ids)):
            aruco.drawAxis(frame, camera_matrix,
                           dist_coef, rvec[i], tvec[i], aruco_6size)
            cv2.putText(frame, str(ids[i]), tuple(corners[i][0][2]),
                        FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, str(ids[i]), tuple(corners[i][0][2]),
                        FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)
            """ cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                        1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                        1, (0, 0, 0), 1, cv2.LINE_AA) """
            """ drone_pose = drone.get_local_pose()
            x, y, z, roll, pitch, yaw = Camera_api.marker_local_pose(rvec[0][0], tvec[0][0],
                                                                     drone_pose)
            marker_pose = [x, y, z, roll, pitch, yaw] """
    """ else:
        # Сбрасываем roll маркера, чтобы, в случае потери маркера, дрон не продолжал кружиться
        marker_pose[3] = 0
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA) """
    # 4
    corners, ids, _ = aruco.detectMarkers(gray, aruco_4dict,
                                          parameters=parameters,
                                          cameraMatrix=camera_matrix,
                                          distCoeff=dist_coef)
    aruco.drawDetectedMarkers(frame, corners)
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, aruco_4size, camera_matrix,
                                                    dist_coef)
    global marker_pose
    if np.all(ids is not None):
        for i in range(len(ids)):
            print(corners[i][0][2])
            aruco.drawAxis(frame, camera_matrix,
                           dist_coef, rvec[i], tvec[i], aruco_4size)
            cv2.putText(frame, str(ids[i]), tuple(corners[i][0][2]),
                        FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, str(ids[i]), tuple(corners[i][0][2]),
                        FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)

    try:
        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
    except Exception as e:
        rospy.loginfo(e)


# calibration_save.yaml - уже проведена калибровка
camera_matrix, dist_coef = clb.loadCoefficients('calibration_save.yaml')
aruco_4dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
aruco_6dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
aruco_4size = 0.15
aruco_6size = 0.5
parameters = aruco.DetectorParameters_create()

aruco_4pos = {}
aruco_6pos = {}

drone = Drone_api()
""" drone.start()
rospy.loginfo('Drone armed') """
image_sub = rospy.Subscriber('/realsense/color/image_raw',
                             Image, callback, queue_size=1)
rospy.loginfo('Start Subscriber')
image_pub = rospy.Publisher('/realsense/color/location_img',
                            Image, queue_size=1)
rospy.loginfo('Start Publisher')

""" drone.sleep(5)
drone.set_local_pose(0, 0, 2, 0)
while not drone.point_is_reached() and not drone.is_shutdown():
    drone.sleep(0.5) """
# Т.к. после закрытия топика с изображением сохраняется последний кадр,
# необходимо перед стартом основной части скрипта "сбросить" значения маркера
marker_pose = [2, 0, 2, 0, 0, 0]
while not Drone_api.is_shutdown():
    print('OK')
    """ drone_pose = drone.get_local_pose()
    correct_drone_yaw = marker_pose[3] + drone_pose[5]
    correct_drone_x = marker_pose[0] + (-2 * cos(correct_drone_yaw))

    correct_drone_y = marker_pose[1] + (-2 * sin(correct_drone_yaw))
    drone.set_local_pose(correct_drone_x, correct_drone_y, marker_pose[2],
                         correct_drone_yaw) """
    Drone_api.sleep(0.5)
rospy.loginfo('Drone disarmed')
