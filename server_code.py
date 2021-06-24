#!/usr/bin/python
# encoding:utf-8
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import socket

HOST = '10.42.0.1'  # Standard loopback interface address (localhost)
PORT = 60001  # Port to listen on (non-privileged ports are > 1023)

timeBegin = 0
timeEnd = 0

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_twist_keyboard')

    print('Ready for connection at ' + HOST + ' Port ' + str(PORT))

    speed = rospy.get_param("~speed", 0.25)
    turn = rospy.get_param("~turn", 1.5)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    try:
        while 1:
            data = conn.recv(1024)
            if not data:
                break
            data = [float(i) for i in str(data).split(',')]
            left_control, right_control = data[0], data[1]
            # print('left: ' + str(left_control))
            # print('right: ' + str(right_control))
            test = (0, 0, 0, 0)
            x = (left_control + right_control) / 2
            y = test[1]
            z = test[2]
            th = (right_control - left_control) / 2

            twist = Twist()
            twist.linear.x = x * speed;
            twist.linear.y = y * speed;
            twist.linear.z = z * speed;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = th * turn
            pub.publish(twist)
    except Exception as e:
        print(e)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    conn.close()