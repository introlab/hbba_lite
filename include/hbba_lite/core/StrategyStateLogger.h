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

class DummyStrategyStateLogger : public StrategyStateLogger
{
public:
    DummyStrategyStateLogger();
    ~DummyStrategyStateLogger() override = default;

    DECLARE_NOT_COPYABLE(DummyStrategyStateLogger);
    DECLARE_NOT_MOVABLE(DummyStrategyStateLogger);

    void log(DesireType desireType, StrategyType strategyType, bool enabled) override;
};

#endif
