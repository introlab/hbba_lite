#include <hbba_lite/core/StrategyStateLogger.h>

StrategyStateLogger::StrategyStateLogger() {}

NoOpStrategyStateLogger::NoOpStrategyStateLogger() {}

void NoOpStrategyStateLogger::log(DesireType desireType, StrategyType strategyType, bool enabled) {}
