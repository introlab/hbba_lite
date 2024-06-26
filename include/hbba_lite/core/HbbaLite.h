#ifndef HBBA_LITE_CORE_HBBA_LITE_H
#define HBBA_LITE_CORE_HBBA_LITE_H

#include <hbba_lite/utils/ClassMacros.h>

#include <hbba_lite/core/DesireSet.h>
#include <hbba_lite/core/Strategy.h>
#include <hbba_lite/core/Solver.h>
#include <hbba_lite/core/StrategyStateLogger.h>

#include <semaphore.h>

#include <atomic>
#include <unordered_map>
#include <memory>
#include <vector>
#include <typeindex>
#include <thread>
#include <mutex>
#include <set>
#include <optional>

template<>
struct std::hash<std::pair<DesireType, size_t>>
{
    std::size_t operator()(std::pair<DesireType, size_t> const& x) const noexcept
    {
        std::size_t h1 = std::hash<DesireType>()(x.first);
        std::size_t h2 = std::hash<size_t>()(x.second);
        return h1 ^ (h2 << 1);
    }
};

class HbbaLite : public DesireSetObserver
{
    std::shared_ptr<DesireSet> m_desireSet;
    std::unordered_map<DesireType, std::vector<std::unique_ptr<BaseStrategy>>> m_strategiesByDesireType;
    std::unordered_map<std::string, uint16_t> m_resourcesByNames;
    std::unique_ptr<Solver> m_solver;
    std::unique_ptr<StrategyStateLogger> m_strategyStateLogger;

    std::mutex m_pendingDesiresMutex;
    sem_t m_pendingDesiresSemaphore; // TODO remplace with C++20 semaphore
    std::optional<std::vector<std::unique_ptr<Desire>>> m_pendingDesires;

    std::atomic_bool m_stopped;
    std::unique_ptr<std::thread> m_thread;

    mutable std::mutex m_activeDesireNamesMutex;
    std::set<std::string> m_activeDesireNames;
    mutable std::mutex m_activeStrategiesMutex;
    std::set<std::string> m_activeStrategies;

public:
    HbbaLite(
        std::shared_ptr<DesireSet> desireSet,
        std::vector<std::unique_ptr<BaseStrategy>> strategies,
        std::unordered_map<std::string, uint16_t> resourcesByNames,
        std::unique_ptr<Solver> solver,
        std::unique_ptr<StrategyStateLogger> strategyStateLogger = std::make_unique<NoOpStrategyStateLogger>());
    ~HbbaLite();

    DECLARE_NOT_COPYABLE(HbbaLite);
    DECLARE_NOT_MOVABLE(HbbaLite);

    void onDesireSetChanged(const std::vector<std::unique_ptr<Desire>>& enabledDesires) override;
    std::vector<std::string> getActiveStrategies() const;
    std::vector<std::string> getActiveDesireNames() const;

private:
    void checkStrategyResources(
        DesireType desireType,
        const std::unordered_map<std::string, uint16_t>& resourcesByNames);

    void run();
    void updateStrategies(std::vector<std::unique_ptr<Desire>> desires);
    void updateActiveStrategies(
        const std::vector<std::unique_ptr<Desire>>& desires,
        const std::unordered_set<SolverResult>& results);
    void updateActiveDesireNames(
        const std::vector<std::unique_ptr<Desire>>& desires,
        const std::unordered_set<SolverResult>& results);
};

#endif
