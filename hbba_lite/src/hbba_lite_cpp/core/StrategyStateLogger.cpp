#include <hbba_lite/core/StrategyStateLogger.h>

StrategyStateLogger::StrategyStateLogger() {}

NoOpStrategyStateLogger::NoOpStrategyStateLogger() {}

void NoOpStrategyStateLogger::log([[maybe_unused]] DesireType desireType, [[maybe_unused]] StrategyType strategyType, [[maybe_unused]] bool enabled) {}
