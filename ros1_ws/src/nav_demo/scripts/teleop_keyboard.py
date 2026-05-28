#!/usr/bin/env python3
"""
teleop_keyboard.py
键盘遥操作节点 — 发布 /cmd_vel (geometry_msgs/Twist)

控制键:
    W/S: 前进/后退
    A/D: 左移/右移
    Q/E: 左转/右转
    Space: 急停 (归零)
    Ctrl+C: 退出

持续以 10Hz 发布当前速度指令, 按键改变目标速度.
"""

import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

HELP_TEXT = """
========================================
  Keyboard Teleop for Quadruped Robot
========================================
  W/S  : forward / backward
  A/D  : strafe left / right
  Q/E  : turn left / right
  R    : reset robot position
  Space: emergency stop (zero velocity)
  Ctrl+C: quit
----------------------------------------
  Velocities are held until changed.
  Publishing at 10 Hz continuously.
========================================
"""

BINDINGS = {
    'w': ( 1.0,  0.0,  0.0),
    's': (-1.0,  0.0,  0.0),
    'a': ( 0.0,  1.0,  0.0),
    'd': ( 0.0, -1.0,  0.0),
    'q': ( 0.0,  0.0,  1.0),
    'e': ( 0.0,  0.0, -1.0),
    ' ': ( 0.0,  0.0,  0.0),
}


def get_key_nonblocking(timeout=0.1):
    """非阻塞读取单个按键, timeout 秒内无输入返回 ''"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
        else:
            ch = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    rospy.init_node("teleop_keyboard")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    reset_pub = rospy.Publisher("/reset_robot", Empty, queue_size=1)

    linear_speed = rospy.get_param("~linear_speed", 1.0)
    lateral_speed = rospy.get_param("~lateral_speed", 1.0)
    angular_speed = rospy.get_param("~angular_speed", 1.0)

    print(HELP_TEXT)
    print(f"Speeds: linear={linear_speed}, lateral={lateral_speed}, angular={angular_speed}")

    # 当前速度状态 (持续保持直到改变)
    cur_vx, cur_vy, cur_wz = 0.0, 0.0, 0.0

    rate = rospy.Rate(10)  # 10 Hz 持续发布

    try:
        while not rospy.is_shutdown():
            key = get_key_nonblocking(timeout=0.1)

            if key == '\x03':  # Ctrl+C
                break

            if key:
                if key.lower() == 'r':
                    reset_pub.publish(Empty())
                    cur_vx, cur_vy, cur_wz = 0.0, 0.0, 0.0
                    print("\n[RESET] Robot position reset requested")
                else:
                    binding = BINDINGS.get(key.lower())
                    if binding is not None:
                        cur_vx = binding[0] * linear_speed
                        cur_vy = binding[1] * lateral_speed
                        cur_wz = binding[2] * angular_speed

            # 持续发布当前速度
            twist = Twist()
            twist.linear.x = cur_vx
            twist.linear.y = cur_vy
            twist.angular.z = cur_wz
            pub.publish(twist)

            status = f"vx={cur_vx:+.1f} vy={cur_vy:+.1f} wz={cur_wz:+.1f}"
            print(f"\r{status}    ", end="", flush=True)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # 发送停止指令
        pub.publish(Twist())
        print("\nStopped.")


if __name__ == "__main__":
    main()
