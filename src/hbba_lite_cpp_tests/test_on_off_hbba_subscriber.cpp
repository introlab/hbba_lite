#include <rclcpp/rclcpp.hpp>

#include <hbba_lite/filters/Subscribers.h>
#include <hbba_lite/msg/int32_stamped.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("test_on_off_hbba_subscriber");
    OnOffHbbaSubscriber<hbba_lite::msg::Int32Stamped> sub(
        node,
        "int_topic_1",
        10,
        [node](const hbba_lite::msg::Int32Stamped::SharedPtr msg)
        { RCLCPP_INFO(node->get_logger(), "Data received : %i", static_cast<int>(msg->data)); });

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
