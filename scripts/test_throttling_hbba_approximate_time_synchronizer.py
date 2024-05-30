#!/usr/bin/env python3
import rclpy
import rclpy.node

import message_filters

from hbba_lite.msg import Int32Stamped

import hbba_lite


def main():
    rclpy.init()
    node = rclpy.node.Node('test_throttling_hbba_approximate_time_synchronizer')

    def callback(data1, data2):
        node.get_logger().info('Data received : {} {}'.format(data1.data, data2.data))

    sub1 = message_filters.Subscriber(node, Int32Stamped, 'int_topic_1')
    sub2 = message_filters.Subscriber(node, Int32Stamped, 'int_topic_2')
    _ = hbba_lite.ThrottlingHbbaApproximateTimeSynchronizer(node, [sub1, sub2], 10, 0.1, callback, 'int_topics/filter_state')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
