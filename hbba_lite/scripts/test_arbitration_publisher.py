#!/usr/bin/env python3
import rclpy
import rclpy.node

from std_msgs.msg import String


def main():
    rclpy.init()

    node = rclpy.node.Node('test_arbitration_publisher')
    pub = node.create_publisher(String, 'cmd', 10)
    rate = node.declare_parameter('rate', 1.0).get_parameter_value().double_value

    def callback():
        msg = String()
        msg.data = node.get_name()
        pub.publish(msg)

    timer = node.create_timer(1 / rate, callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()
