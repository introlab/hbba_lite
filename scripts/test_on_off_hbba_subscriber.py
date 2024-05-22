#!/usr/bin/env python3
import rclpy
import rclpy.node

from hbba_lite.msg import Int32Stamped

import hbba_lite


def main():
    rclpy.init()
    node = rclpy.node.Node('test_on_off_hbba_subscriber')

    def callback(data):
        node.get_logger().info('Data received : {}'.format(data.data))

    _ = hbba_lite.OnOffHbbaSubscriber(node, Int32Stamped, 'int_topic_1', callback, 10)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
