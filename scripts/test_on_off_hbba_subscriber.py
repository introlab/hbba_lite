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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
