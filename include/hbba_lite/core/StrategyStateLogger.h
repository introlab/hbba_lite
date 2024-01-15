#ifndef HBBA_LITE_CORE_STRATEGY_STATE_LOGGER_H
#define HBBA_LITE_CORE_STRATEGY_STATE_LOGGER_H

#include <hbba_lite/core/Desire.h>
#include <hbba_lite/core/Strategy.h>
#include <hbba_lite/utils/ClassMacros.h>

class StrategyStateLogger
{
public:
    StrategyStateLogger();
    virtual ~StrategyStateLogger() = default;

    DECLARE_NOT_COPYABLE(StrategyStateLogger);
    DECLARE_NOT_MOVABLE(StrategyStateLogger);

    virtual void log(DesireType desireType, StrategyType strategyType, bool enabled) = 0;
};

class NoOpStrategyStateLogger : public StrategyStateLogger
{
public:
    NoOpStrategyStateLogger();
    ~NoOpStrategyStateLogger() override = default;

    DECLARE_NOT_COPYABLE(NoOpStrategyStateLogger);
    DECLARE_NOT_MOVABLE(NoOpStrategyStateLogger);

    void log(DesireType desireType, StrategyType strategyType, bool enabled) override;
};

#endif
