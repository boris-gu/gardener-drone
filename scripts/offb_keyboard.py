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


def state_cb(state):
    global current_state
    current_state = state


def pos_cb(pose):
    global current_yaw
    current_yaw = euler_from_quaternion([pose.pose.orientation.x,
                                         pose.pose.orientation.y,
                                         pose.pose.orientation.z,
                                         pose.pose.orientation.w])[2]


def change_velocity(x, y, z, yaw):
    new_vel = Twist()
    new_vel.linear.x = x
    new_vel.linear.y = y
    new_vel.linear.z = z
    new_vel.angular.z = yaw
    return new_vel


def offb_state():
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


if __name__ == '__main__':
    speed_xy = 4  # м/с
    speed_z = 1.5
    speed_yaw = pi  # рад/с

    preset_vel = [0, 0, 0, 0]
    current_state = State()
    current_yaw = 0

    # КЛАВИАТУРА
    pressed_keys = {keyboard.Key.up: False,
                    keyboard.Key.down: False,
                    keyboard.Key.right: False,
                    keyboard.Key.left: False,
                    'w': False,
                    's': False,
                    'd': False,
                    'a': False}

    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()

    # ДРОН
    rospy.init_node('offb_node', anonymous=True)
    # Подписчик состояния, издатель скорости, клиенты для смены режима и арма
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

    thread_state = Thread(target=offb_state, daemon=True)
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
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
