#!/usr/bin/env python3
import rclpy
import rclpy.node

from std_msgs.msg import Int8

import hbba_lite


def main():
    rclpy.init()

    node = rclpy.node.Node('test_on_off_hbba_publisher')
    pub = hbba_lite.OnOffHbbaPublisher(node, Int8, 'int_topic_1', 10)

    i = 0

    def callback():
        nonlocal i

        msg = Int8()
        msg.data = i
        pub.publish(msg)

        i += 1

    timer = node.create_timer(1, callback)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
