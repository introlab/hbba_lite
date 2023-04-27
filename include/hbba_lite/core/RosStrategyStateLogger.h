#ifndef HBBA_LITE_CORE_ROS_STRATEGY_STATE_LOGGER_H
#define HBBA_LITE_CORE_ROS_STRATEGY_STATE_LOGGER_H

#include <hbba_lite/core/StrategyStateLogger.h>

#include <ros/ros.h>

class RosStrategyStateLogger : public StrategyStateLogger
{
    ros::NodeHandle& m_nodeHandle;
    ros::Publisher m_strategyStatePub;

public:
    RosStrategyStateLogger(ros::NodeHandle& nodeHandle);

    DECLARE_NOT_COPYABLE(RosStrategyStateLogger);
    DECLARE_NOT_MOVABLE(RosStrategyStateLogger);

    void log(DesireType desireType, StrategyType strategyType, bool enabled) override;
};

#endif
