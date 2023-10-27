#!/usr/bin/env python3
#
# usage:
#   python3 drone_circular_move.py
import rospy
from geometry_msgs.msg import Twist
import time


def main():
    rospy.init_node('left_foward_move', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(30)

    # 创建 Twist 消息
    twist = Twist()
    while not rospy.is_shutdown():
        # 向左前方前进 3 秒
        twist.linear.x = 0.3
        twist.linear.y = 0.15
        pub.publish(twist)
        time.sleep(4)

        # 平滑停止 1 秒
        for _ in range(10):
            twist.linear.x -= 0.03
            twist.linear.y -= 0.015
            pub.publish(twist)  # 发布频率为 10Hz
            time.sleep(0.1)

        # # 停止 1 秒
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # pub.publish(twist)
        # time.sleep(1)

        # 向右后方后退 3 秒
        twist.linear.x = -0.3
        twist.linear.y = -0.15
        pub.publish(twist)
        time.sleep(4)

        for _ in range(10):
            twist.linear.x += 0.03
            twist.linear.y += 0.015
            pub.publish(twist)
            time.sleep(0.1)

        # # 停止 1 秒
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # pub.publish(twist)
        # time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
