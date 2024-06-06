#include <hbba_lite/core/RosStrategyStateLogger.h>

using namespace std;

RosLogStrategyStateLogger::RosLogStrategyStateLogger(std::shared_ptr<rclcpp::Node> node) : m_node(move(node)) {}

void RosLogStrategyStateLogger::log(DesireType desireType, StrategyType strategyType, bool enabled)
{
    RCLCPP_INFO_STREAM(
        m_node->get_logger(),
        "HBBA strategy state changed: "
            << "( " << desireType.name() << ", " << strategyType.name() << ") -> "
            << (enabled ? "enabled" : "disabled"));
}


RosTopicStrategyStateLogger::RosTopicStrategyStateLogger(std::shared_ptr<rclcpp::Node> node) : m_node(move(node))
{
    m_strategyStatePub = m_node->create_publisher<hbba_lite_msgs::msg::StrategyState>("hbba_strategy_state_log", 1000);
}

void RosTopicStrategyStateLogger::log(DesireType desireType, StrategyType strategyType, bool enabled)
{
    hbba_lite_msgs::msg::StrategyState msg;
    msg.desire_type_name = desireType.name();
    msg.strategy_type_name = strategyType.name();
    msg.enabled = enabled;
    m_strategyStatePub->publish(msg);
}
