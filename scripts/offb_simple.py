#!/usr/bin/env python3

# =============
#  Взлет дрона
# =============

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# Обратный вызов для сохранения состояния дрона
current_state = State()


def state_cb(state):
    global current_state
    current_state = state


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
    rate = rospy.Rate(20.0)

    # Ожидаем связь между МАВРОС и автопилотом
    while not rospy.is_shutdown() and not current_state.connected:
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as err:
            pass

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Необходимо начать потоковую передачу заданных значений до установки режима OFFBOARD
    # 100 - произвольное число
    for i in range(100):
        local_pos_pub.publish(pose)
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as err:
            pass

    last_request = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if current_state.mode != 'OFFBOARD' and (now - last_request > rospy.Duration(5)):
            set_mode_client(base_mode=0, custom_mode='OFFBOARD')
            rospy.loginfo('Offboard enabled')
            last_request = now
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5)):
                arming_client(True)
                rospy.loginfo('Vehicle armed')
                last_request = now

        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException as err:
            pass
