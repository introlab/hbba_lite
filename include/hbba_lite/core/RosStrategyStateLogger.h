#ifndef HBBA_LITE_CORE_ROS_STRATEGY_STATE_LOGGER_H
#define HBBA_LITE_CORE_ROS_STRATEGY_STATE_LOGGER_H

#include <hbba_lite/core/StrategyStateLogger.h>

#include <ros/ros.h>

class RosLogStrategyStateLogger : public StrategyStateLogger
{
public:
    RosLogStrategyStateLogger() = default;
    ~RosLogStrategyStateLogger() override = default;

    DECLARE_NOT_COPYABLE(RosLogStrategyStateLogger);
    DECLARE_NOT_MOVABLE(RosLogStrategyStateLogger);

    void log(DesireType desireType, StrategyType strategyType, bool enabled) override;
};


class RosTopicStrategyStateLogger : public StrategyStateLogger
{
    ros::NodeHandle& m_nodeHandle;
    ros::Publisher m_strategyStatePub;

public:
    RosTopicStrategyStateLogger(ros::NodeHandle& nodeHandle);
    ~RosTopicStrategyStateLogger() override = default;

    DECLARE_NOT_COPYABLE(RosTopicStrategyStateLogger);
    DECLARE_NOT_MOVABLE(RosTopicStrategyStateLogger);

    void log(DesireType desireType, StrategyType strategyType, bool enabled) override;
};

#endif
