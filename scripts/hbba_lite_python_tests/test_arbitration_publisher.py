#!/usr/bin/env python3
import rospy

from std_msgs.msg import String


def main():
    rospy.init_node('test_arbitration_publisher')
    pub = rospy.Publisher('cmd', String, queue_size=10)

    rate = rospy.Rate(rospy.get_param('~rate', 1.0))
    while not rospy.is_shutdown():

        msg = String()
        msg.data = rospy.get_name()
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
