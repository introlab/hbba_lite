#!/usr/bin/env python3
import rclpy
import rclpy.node

from std_msgs.msg import Header
from hbba_lite_msgs.msg import Int32Stamped


def main():
    rclpy.init()

    node = rclpy.node.Node('test_on_off_hbba_publisher')

    pub1 = node.create_publisher(Int32Stamped, 'int_topic_1', 10)
    pub2 = node.create_publisher(Int32Stamped, 'int_topic_2', 10)
    i = 0

    def callback():
        nonlocal i

        header = Header()
        header.stamp = node.get_clock().now().to_msg()

        msg1 = Int32Stamped()
        msg1.header = header
        msg1.data = 2 * i

        msg2 = Int32Stamped()
        msg2.header = header
        msg2.data = 4 * i

        pub1.publish(msg1)
        pub2.publish(msg2)

        i += 1

    timer = node.create_timer(1, callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()
