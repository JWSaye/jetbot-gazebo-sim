#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys
import select
import termios
import tty

msg = """
Control Your JetBot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

c : collect data (when in data collection mode)

CTRL-C to quit
"""

JETBOT_MAX_LIN_VEL = 0.2
JETBOT_MAX_ANG_VEL = 2.0

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0:.03f}\t angular velocity {1:.03f} '.format(
        target_linear_velocity,
        target_angular_velocity))

def constrain(input, low, high):
    if input < low:
        return low
    elif input > high:
        return high
    return input

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -JETBOT_MAX_LIN_VEL, JETBOT_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -JETBOT_MAX_ANG_VEL, JETBOT_MAX_ANG_VEL)

def main():
    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    key_pub = rospy.Publisher('keys', String, queue_size=10)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = get_key()

            if key:
                key_msg = String()
                key_msg.data = str(key)
                key_pub.publish(key_msg)

            if key == 'w':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
            elif key == 'x':
                target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
            elif key == 'a':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
            elif key == 'd':
                target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()
            twist.linear.x = control_linear_velocity = target_linear_velocity
            twist.angular.z = control_angular_velocity = target_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

if __name__ == '__main__':
    main()