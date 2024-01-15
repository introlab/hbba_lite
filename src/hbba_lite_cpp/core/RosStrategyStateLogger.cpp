#include <hbba_lite/core/RosStrategyStateLogger.h>

#include <hbba_lite/StrategyState.h>

using namespace std;

void RosLogStrategyStateLogger::log(DesireType desireType, StrategyType strategyType, bool enabled)
{
    ROS_INFO_STREAM(
        "HBBA strategy state changed: "
        << "( " << desireType.name() << ", " << strategyType.name() << ") -> " << (enabled ? "enabled" : "disabled"));
}


RosTopicStrategyStateLogger::RosTopicStrategyStateLogger(ros::NodeHandle& nodeHandle) : m_nodeHandle(nodeHandle)
{
    m_strategyStatePub = m_nodeHandle.advertise<hbba_lite::StrategyState>("hbba_strategy_state_log", 1000);
}

void RosTopicStrategyStateLogger::log(DesireType desireType, StrategyType strategyType, bool enabled)
{
    hbba_lite::StrategyState msg;
    msg.desire_type_name = desireType.name();
    msg.strategy_type_name = strategyType.name();
    msg.enabled = enabled;
    m_strategyStatePub.publish(msg);
}
