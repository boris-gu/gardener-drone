#!/usr/bin/env python3

# =============
#  Взлет дрона
# =============

import rospy
from math import pi, atan2
from threading import Thread
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler

# ==============================
# Переменные для редактирования
# ==============================
acceptable_error = 0.4  # Приемлемая ошибка при подлете к точке
start_alt = 1           # Начальная (и конечная) высота
points = [[0, 0, 5],
          [0, 4, 8],
          [4, 4, 5],
          [4, 0, 8]] * 2 + [[0, 0, 5]]
yaws = [pi, pi/2, 0, pi/(-2)] * 2 + [pi]


# Обратный вызов для сохранения состояния, положения дрона и начальной позиции
current_state = State()
current_pose = PoseStamped()
start_pose = None  # Начальная (и конечная) точка


def state_cb(state):
    global current_state
    current_state = state


def local_pos(pose):
    global current_pose
    current_pose = pose
    global start_pose
    if start_pose is None:
        start_pose = current_pose
        start_pose.pose.position.z = start_alt


def change_pose(point, yaw=None, head_first=False):  # head_first в приоритете
    new_pose = PoseStamped()
    new_pose.pose.position.x = point[0]
    new_pose.pose.position.y = point[1]
    new_pose.pose.position.z = point[2]
    if head_first:
        # Подсчет угла
        x = point[0] - current_pose.pose.position.x
        y = point[1] - current_pose.pose.position.y
        yaw = atan2(y, x)
    if yaw is not None:
        q = quaternion_from_euler(0, 0, yaw)
        new_pose.pose.orientation.x = q[0]
        new_pose.pose.orientation.y = q[1]
        new_pose.pose.orientation.z = q[2]
        new_pose.pose.orientation.w = q[3]
    return new_pose


def distance(pose, point):
    d = (pose.pose.position.x - point[0])**2
    d += (pose.pose.position.y - point[1])**2
    d += (pose.pose.position.z - point[2])**2
    return d ** 0.5


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
    # Подписчики состояния и локального плоложения, издатель локальной позиции,
    # клиенты для смены режима и арма
    state_sub = rospy.Subscriber('mavros/state', State,
                                 state_cb, queue_size=10)
    local_pos_sub = rospy.Subscriber('/mavros/local_position/pose',
                                     PoseStamped, local_pos, queue_size=10)
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local',
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
        while start_pose is None:
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
        # Взлет
        start_pose.header.stamp = rospy.Time.now()
        rospy.loginfo('TAKEOFF')
        start_point = [start_pose.pose.position.x,
                       start_pose.pose.position.y,
                       start_pose.pose.position.z]
        while distance(current_pose, start_point) > acceptable_error:
            local_pos_pub.publish(start_pose)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
        # Полет по квадрату
        for point, yaw in zip(points, yaws):
            # pose = change_pose(point, yaw)  # Установка yaw как в points_offb
            pose = change_pose(point, head_first=True)
            pose.header.stamp = rospy.Time.now()
            rospy.loginfo(f'To point {point}')
            while distance(current_pose, point) > acceptable_error:
                local_pos_pub.publish(pose)
                try:
                    rate.sleep()
                except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                    pass
        # Посадка
        pose = change_pose(start_point)
        pose.header.stamp = rospy.Time.now()
        rospy.loginfo('LANDING')
        while distance(current_pose, start_point) > acceptable_error:
            local_pos_pub.publish(start_pose)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
    except rospy.ROSInterruptException:
        pass
