#ifndef HBBA_LITE_CORE_ROS_STRATEGY_STATE_LOGGER_H
#define HBBA_LITE_CORE_ROS_STRATEGY_STATE_LOGGER_H

#include <hbba_lite/core/StrategyStateLogger.h>

#include <rclcpp/rclcpp.hpp>

#include <hbba_lite_msgs/msg/strategy_state.hpp>

class RosLogStrategyStateLogger : public StrategyStateLogger
{
    std::shared_ptr<rclcpp::Node> m_node;

public:
    explicit RosLogStrategyStateLogger(std::shared_ptr<rclcpp::Node> node);
    ~RosLogStrategyStateLogger() override = default;

    DECLARE_NOT_COPYABLE(RosLogStrategyStateLogger);
    DECLARE_NOT_MOVABLE(RosLogStrategyStateLogger);

    void log(DesireType desireType, StrategyType strategyType, bool enabled) override;
};


class RosTopicStrategyStateLogger : public StrategyStateLogger
{
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Publisher<hbba_lite_msgs::msg::StrategyState>::SharedPtr m_strategyStatePub;

public:
    explicit RosTopicStrategyStateLogger(std::shared_ptr<rclcpp::Node> node);
    ~RosTopicStrategyStateLogger() override = default;

    DECLARE_NOT_COPYABLE(RosTopicStrategyStateLogger);
    DECLARE_NOT_MOVABLE(RosTopicStrategyStateLogger);

    void log(DesireType desireType, StrategyType strategyType, bool enabled) override;
};

#endif
