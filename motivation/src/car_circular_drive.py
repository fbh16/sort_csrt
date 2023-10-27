#!/usr/bin/env python3
#
# usage:
#   python3 car_circular_drive.py
#
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('square_navigation')
    pub = rospy.Publisher('/tianbot_mini/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 发布频率为 1Hz

    while not rospy.is_shutdown():
        # 前进
        for _ in range(5):
            twist = Twist()
            twist.linear.x = 0.5
            pub.publish(twist)
            rate.sleep()

        # 停止
        twist = Twist()
        pub.publish(twist)
        rospy.sleep(1)

        # 右转
        for _ in range(5):
            twist = Twist()
            twist.angular.z = 0.3
            pub.publish(twist)
            rate.sleep()

        # 停止
        twist = Twist()
        pub.publish(twist)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
