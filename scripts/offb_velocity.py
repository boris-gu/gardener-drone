#!/usr/bin/env python3

# =============
#  Взлет дрона
# =============

import rospy
from math import pi, cos, sin
from threading import Thread
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

speed = 5  # м/с

# Обратный вызов для сохранения состояния дрона
current_state = State()

start_vel = [0, 0, 1]
angles = [i/180*pi for i in range(0, 360, 5)] * 2
vel_x = [cos(i)*speed for i in angles]
vel_y = [sin(i)*speed for i in angles]
vel_z_up = 1
vel_z_down = -1
vel_yaw = 2*pi/7.2
# Почему 2pi/7.2 рад/с
# На 1 круг тратится 72 команды, которые отсылаются по два раза с частотой 20Гц
# Т.е. на выполнение одной команды уходит 0.1 секунда,
# а на выполнение всех 72 команд (круг) - 7.2 секунды
# Так должно быть в идеале, но из-за физики реальная угловая (и линейная) скорость
# может отличаться от заданной, поэтому в конце скрипта коптер может не находиться
# в первоначальном положении
# ИЗ-ЗА ЭТОГО РЕЗУЛЬТАТ СКРИПТА НЕПРЕДСКАЗУЕМ


def state_cb(state):
    global current_state
    current_state = state


def change_velocity(x, y, z, yaw=None):
    new_vel = Twist()
    new_vel.linear.x = x
    new_vel.linear.y = y
    new_vel.linear.z = z
    if yaw is not None:
        new_vel.angular.z = yaw
    return new_vel


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
    # Подписчик состояния, издатель скорости, клиенты для смены режима и арма
    state_sub = rospy.Subscriber('mavros/state', State,
                                 state_cb, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
                                    Twist, queue_size=10)
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
        # Подъем
        vel = change_velocity(*start_vel)
        for i in range(200):  # 200 = 10 секунд, тк частота = 20
            if rospy.is_shutdown():
                break
            local_vel_pub.publish(vel)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
        # Полет по спирали вверх
        for x, y in zip(vel_x, vel_y):
            vel = change_velocity(x, y, vel_z_up, vel_yaw)
            for i in range(2):
                local_vel_pub.publish(vel)
                try:
                    rate.sleep()
                except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                    pass
        # Полет по спирали вниз
        for x, y in zip(vel_x, vel_y):
            vel = change_velocity(x, y, vel_z_down, vel_yaw)
            for i in range(2):
                local_vel_pub.publish(vel)
                try:
                    rate.sleep()
                except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                    pass
        # "Замыкание" круга
        vel = change_velocity(1, 0, 0, vel_yaw)
        for i in range(2):
            local_vel_pub.publish(vel)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
        # Зависнуть
        vel = change_velocity(0, 0, 0)
        for i in range(200):
            if rospy.is_shutdown():
                break
            local_vel_pub.publish(vel)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as err:
                pass
    except rospy.ROSInterruptException:
        pass
