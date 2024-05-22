#include <rclcpp/rclcpp.hpp>

#include <hbba_lite/filters/HbbaFilterNode.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ThrottlingHbbaFilterNode>("throttling_hbba_filter_node");
    node->run();

    rclcpp::shutdown();

    return 0;
}
