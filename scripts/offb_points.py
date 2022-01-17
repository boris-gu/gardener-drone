#!/usr/bin/env python3

# =============
#  Взлет дрона
# =============

import rospy
from math import pi
from threading import Thread
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler

start_point = [0, 0, 2]
points = [[0, 0, 5],
          [0, 4, 8],
          [4, 4, 5],
          [4, 0, 8]] * 2 + [[0, 0, 5]]
yaws = [pi, pi/2, 0, pi/(-2)] * 2 + [pi]

# Обратный вызов для сохранения состояния дрона
current_state = State()


def state_cb(state):
    global current_state
    current_state = state


def change_pose(point, yaw=None):
    new_pose = PoseStamped()
    new_pose.pose.position.x = point[0]
    new_pose.pose.position.y = point[1]
    new_pose.pose.position.z = point[2]
    if yaw:
        q = quaternion_from_euler(0, 0, yaw)
        new_pose.pose.orientation.x = q[0]
        new_pose.pose.orientation.y = q[1]
        new_pose.pose.orientation.z = q[2]
        new_pose.pose.orientation.w = q[3]
    return new_pose


def check_state():
    rate = rospy.Rate(1)
    prev_state = current_state
    while not rospy.is_shutdown():
        try:
            if current_state.mode != 'OFFBOARD':
                rospy.loginfo('Trying to change state to OFFBOARD')
                set_mode_client(base_mode=0, custom_mode='OFFBOARD')
            else:
                if not current_state.armed:
                    rospy.loginfo('Trying to arm')
                    arming_client(True)
            if prev_state.mode != current_state.mode:
                rospy.loginfo(current_state.mode)
            if prev_state.armed != current_state.armed:
                rospy.loginfo('ARMED')
            prev_state = current_state
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    rospy.init_node('offb_node', anonymous=True)
    # Подписчик состояния, издатель локальной позиции, клиенты для смены режима и арма
    state_sub = rospy.Subscriber('mavros/state', State,
                                 state_cb, queue_size=10)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local',
                                    PoseStamped, queue_size=10)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    # Установка скорости публикации выше 2Гц
    rate = rospy.Rate(20)

    # Ожидаем связь между МАВРОС и автопилотом
    while not rospy.is_shutdown() and not current_state.connected:
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as err:
            pass

    # Дрон не перейдет в режим OFFBOARD, пока не начнется потоковая передача значений
    thread_state = Thread(target=check_state, daemon=True)
    thread_state.start()

    try:
        # Висение на месте
        pose = change_pose(start_point)
        pose.header.stamp = rospy.Time.now()
        for i in range(200):  # 200 = 10 секунд, тк частота = 20
            if rospy.is_shutdown():
                break
            local_pos_pub.publish(pose)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
        # Полет по квадрату
        for point, yaw in zip(points, yaws):
            if rospy.is_shutdown():
                break
            pose = change_pose(point, yaw)
            pose.header.stamp = rospy.Time.now()
            rospy.loginfo(f'To point {point}')
            for i in range(80):
                local_pos_pub.publish(pose)
                try:
                    rate.sleep()
                except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                    pass
        # Висение на месте
        pose = change_pose(start_point)
        pose.header.stamp = rospy.Time.now()
        for i in range(200):  # 200 = 10 секунд, тк частота = 20Гц
            if rospy.is_shutdown():
                break
            local_pos_pub.publish(pose)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
    except rospy.ROSInterruptException:
        pass
