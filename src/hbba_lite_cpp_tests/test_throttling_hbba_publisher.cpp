#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

#include <hbba_lite/filters/Publishers.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("test_on_off_hbba_publisher");
    ThrottlingHbbaPublisher<std_msgs::msg::Int8> pub(node, "int_topic", 10);

    rclcpp::Rate rate(1);
    for (int i = 0; rclcpp::ok(); i++)
    {
        std_msgs::msg::Int8 msg;
        msg.data = i;
        pub.publish(msg);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
