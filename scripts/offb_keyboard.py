#!/usr/bin/env python3

# =============
#  Взлет дрона
# =============

import rospy
from math import pi, cos, sin, sqrt
from threading import Thread
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import euler_from_quaternion
from pynput import keyboard
import curses


def main(stdscr):
    def state_cb(state):
        global current_state
        current_state = state

    def pos_cb(pose):
        global current_yaw
        global current_position
        current_yaw = euler_from_quaternion([pose.pose.orientation.x,
                                            pose.pose.orientation.y,
                                            pose.pose.orientation.z,
                                            pose.pose.orientation.w])[2]
        current_position = pose.pose.position

    def change_velocity(x, y, z, yaw):
        new_vel = Twist()
        new_vel.linear.x = x
        new_vel.linear.y = y
        new_vel.linear.z = z
        new_vel.angular.z = yaw
        return new_vel

    def fun_state():
        rate = rospy.Rate(1)
        prev_state = current_state
        while not rospy.is_shutdown():
            try:
                if current_state.mode != 'OFFBOARD':
                    set_mode_client(base_mode=0, custom_mode='OFFBOARD')
                else:
                    if not current_state.armed:
                        arming_client(True)
                if prev_state.mode != current_state.mode:
                    arming_client(True)
                if prev_state.armed != current_state.armed:
                    pass
                prev_state = current_state
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def on_press(key):
        try:
            k = key.char
            if k in pressed_keys and not pressed_keys[k]:
                pressed_keys[k] = True
                # Z
                if k == 'w':
                    preset_vel[2] = speed_z
                elif k == 's':
                    preset_vel[2] = -speed_z
                # YAW
                elif k == 'd':
                    preset_vel[3] = -speed_yaw
                elif k == 'a':
                    preset_vel[3] = speed_yaw
        except AttributeError:
            if key in pressed_keys and not pressed_keys[key]:
                pressed_keys[key] = True
                # X
                if key == keyboard.Key.up:
                    preset_vel[0] = speed_xy
                elif key == keyboard.Key.down:
                    preset_vel[0] = -speed_xy
                # Y
                elif key == keyboard.Key.right:
                    preset_vel[1] = -speed_xy
                elif key == keyboard.Key.left:
                    preset_vel[1] = speed_xy

    def on_release(key):
        try:
            k = key.char
            if k in pressed_keys and pressed_keys[k]:
                pressed_keys[k] = False
                # Z
                if k in ('w', 's'):
                    if not pressed_keys['w'] and not pressed_keys['s']:
                        preset_vel[2] = 0
                    elif pressed_keys['w']:
                        preset_vel[2] = speed_z
                    elif pressed_keys['s']:
                        preset_vel[2] = -speed_z
                # YAW
                elif k in ('d', 'a'):
                    if not pressed_keys['d'] and not pressed_keys['a']:
                        preset_vel[3] = 0
                    elif pressed_keys['d']:
                        preset_vel[3] = -speed_yaw
                    elif pressed_keys['a']:
                        preset_vel[3] = speed_yaw
        except AttributeError:
            if key in pressed_keys and pressed_keys[key]:
                pressed_keys[key] = False
                # X
                if key in (keyboard.Key.up, keyboard.Key.down):
                    if not pressed_keys[keyboard.Key.up] and not pressed_keys[keyboard.Key.down]:
                        preset_vel[0] = 0
                    elif pressed_keys[keyboard.Key.up]:
                        preset_vel[0] = speed_xy
                    elif pressed_keys[keyboard.Key.down]:
                        preset_vel[0] = -speed_xy
                # Y
                elif key in (keyboard.Key.right, keyboard.Key.left):
                    if not pressed_keys[keyboard.Key.right] and not pressed_keys[keyboard.Key.left]:
                        preset_vel[1] = 0
                    elif pressed_keys[keyboard.Key.right]:
                        preset_vel[1] = -speed_xy
                    elif pressed_keys[keyboard.Key.left]:
                        preset_vel[1] = speed_xy

    # CURSES
    curses.curs_set(0)  # скрыть курсор
    curses.use_default_colors()  # дефолтные цвета
    curses.init_pair(1, -1, 2)
    curses.init_pair(2, 2, -1)
    stick1 = curses.newwin(5, 10, 1, 2)
    stick2 = curses.newwin(5, 10, 1, 14)
    info_win = curses.newwin(5, 22, 7, 2)

    # Постоянная графика
    stick1.border()
    stick2.border()
    info_win.border()
    info_win.addstr(0, 2, 'INFO')
    info_win.addstr(2, 2, 'X:       Y:')
    info_win.addstr(3, 2, 'ALTITUDE:')
    stick1.refresh()
    stick2.refresh()
    info_win.refresh()

    # KEYBOARD
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    # ДРОН
    rospy.init_node('offb_node', anonymous=True)
    # Подписчики состояния и положения, издатель скорости, клиенты для смены режима и арма
    state_sub = rospy.Subscriber('mavros/state', State,
                                 state_cb, queue_size=10)
    local_pos_sub = rospy.Subscriber('/mavros/local_position/pose',
                                     PoseStamped, pos_cb, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
                                    Twist, queue_size=10)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    thread_state = Thread(target=fun_state, daemon=True)
    thread_state.start()

    while not rospy.is_shutdown():
        try:
            if abs(preset_vel[0]) == speed_xy and abs(preset_vel[1]) == speed_xy:
                preset_vel[0] = preset_vel[0] * sqrt(2)/2
                preset_vel[1] = preset_vel[1] * sqrt(2)/2
            # Поворот координат
            x = preset_vel[0]*cos(current_yaw) - preset_vel[1]*sin(current_yaw)
            y = preset_vel[0]*sin(current_yaw) + preset_vel[1]*cos(current_yaw)
            vel = change_velocity(x, y, preset_vel[2], preset_vel[3])
            local_vel_pub.publish(vel)

            # Левый стик
            st1_x = 0
            if preset_vel[3] > 0:
                st1_x = -2
            if preset_vel[3] < 0:
                st1_x = 2
            st1_x += 4
            st1_y = 0
            if preset_vel[2] > 0:
                st1_y = -1
            if preset_vel[2] < 0:
                st1_y = 1
            st1_y += 2
            for i in range(1, 4):
                stick1.addstr(i, 1, '        ')
            stick1.addstr(st1_y, st1_x, '  ', curses.color_pair(1))
            stick1.refresh()

            # Правый стик
            st2_x = 0
            if preset_vel[1] > 0:
                st2_x = -2
            if preset_vel[1] < 0:
                st2_x = 2
            st2_x += 4
            st2_y = 0
            if preset_vel[0] > 0:
                st2_y = -1
            if preset_vel[0] < 0:
                st2_y = 1
            st2_y += 2
            for i in range(1, 4):
                stick2.addstr(i, 1, '        ')
            stick2.addstr(st2_y, st2_x, '  ', curses.color_pair(1))
            stick2.refresh()

            # Информация
            if current_position != None:
                for i in range(12, 21):
                    info_win.addstr(3, i, ' ')
                info_win.addstr(
                    3, 12, f'{current_position.z:.2f}', curses.color_pair(2))
                for i in range(5, 11):
                    info_win.addstr(2, i, ' ')
                info_win.addstr(
                    2, 5, f'{current_position.x:.2f}', curses.color_pair(2))
                for i in range(14, 21):
                    info_win.addstr(2, i, ' ')
                info_win.addstr(
                    2, 14, f'{current_position.y:.2f}', curses.color_pair(2))
            info_win.refresh()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    speed_xy = 4  # м/с
    speed_z = 1.5
    speed_yaw = pi  # рад/с

    preset_vel = [0, 0, 0, 0]
    current_state = State()
    current_yaw = 0
    current_position = None

    # КЛАВИАТУРА
    pressed_keys = {keyboard.Key.up: False,
                    keyboard.Key.down: False,
                    keyboard.Key.right: False,
                    keyboard.Key.left: False,
                    'w': False,
                    's': False,
                    'd': False,
                    'a': False}

    curses.wrapper(main)
