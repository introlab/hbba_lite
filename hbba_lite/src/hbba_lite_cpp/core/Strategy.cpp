#include <hbba_lite/core/Strategy.h>

#include <limits>

using namespace std;

StrategyType::StrategyType(type_index type) : m_type(type) {}

FilterConfiguration::FilterConfiguration()
    : m_type(FilterType::ON_OFF),
      m_rate(0),
      m_defaultState(FilterConfiguration::DefaultState::ENABLED)
{
}

FilterConfiguration::FilterConfiguration(FilterConfiguration::DefaultState defaultState)
    : m_type(FilterType::ON_OFF),
      m_rate(0),
      m_defaultState(defaultState)
{
}

FilterConfiguration::FilterConfiguration(uint16_t rate, FilterConfiguration::DefaultState defaultState)
    : m_type(FilterType::THROTTLING),
      m_rate(rate),
      m_defaultState(defaultState)
{
    if (rate == 0)
    {
        throw HbbaLiteException("Invalid rate (rate=" + to_string(rate) + ")");
    }
}

void FilterPool::add(const string& name, FilterType type)
{
    lock_guard<recursive_mutex> lock(m_mutex);

    auto it = m_typesByName.find(name);
    if (it == m_typesByName.end())
    {
        m_typesByName[name] = type;
        m_countsByName[name] = 0;
    }
    else if (it->second != type)
    {
        throw HbbaLiteException("Not matching type for the filter (" + name + ")");
    }
}

void FilterPool::enable(const StrategyType& type, const string& name, const FilterConfiguration& configuration)
{
    lock_guard<recursive_mutex> lock(m_mutex);

    auto it = m_countsByName.find(name);
    if (it == m_countsByName.end())
    {
        throw HbbaLiteException("Not existing filter (" + name + ")");
    }
    if (m_typesByName[name] != configuration.type())
    {
        throw HbbaLiteException("Not compatible filter configuration (" + name + ")");
    }

    // We don't want to enable the filter if it is already enabled by the strategy.
    if (m_enabledFilterStrategies.count({name, type}) != 0)
    {
        return;
    }

    if (it->second == 0)
    {
        applyEnabling(name, configuration);
        m_lastFilterConfigurationByName[name] = configuration;
    }
    else if (configuration != m_lastFilterConfigurationByName[name])
    {
        throw HbbaLiteException("Not compatible filter configuration (" + name + ")");
    }

    m_enabledFilterStrategies.insert({name, type});

    it->second++;
}

void FilterPool::disable(const StrategyType& type, const string& name)
{
    lock_guard<recursive_mutex> lock(m_mutex);

    // We don't want to decrement the counter if the filter is not enabled by the strategy.
    if (m_enabledFilterStrategies.count({name, type}) == 0)
    {
        return;
    }

    auto it = m_countsByName.find(name);
    if (it == m_countsByName.end())
    {
        throw HbbaLiteException("Not existing filter (" + name + ")");
    }


    m_enabledFilterStrategies.erase({name, type});

    it->second--;
    if (it->second == 0)
    {
        applyDisabling(name);
    }
}

void FilterPool::callApplyEnabling(
    FilterPool& filterPool,
    const std::string& name,
    const FilterConfiguration& configuration)
{
    filterPool.applyEnabling(name, configuration);
}

void FilterPool::callApplyDisabling(FilterPool& filterPool, const std::string& name)
{
    filterPool.applyDisabling(name);
}

BaseStrategy::BaseStrategy(
    uint16_t utility,
    unordered_map<string, uint16_t> resourcesByName,
    unordered_map<string, FilterConfiguration> filterConfigurationsByName,
    shared_ptr<FilterPool> filterPool)
    : m_enabled(false),
      m_desireId(nullopt),
      m_utility(utility),
      m_resourcesByName(move(resourcesByName)),
      m_filterConfigurationsByName(move(filterConfigurationsByName)),
      m_filterPool(move(filterPool))
{
    for (auto& pair : m_filterConfigurationsByName)
    {
        m_filterPool->add(pair.first, pair.second.type());
    }
}

void BaseStrategy::onEnabling(const Desire& desire)
{
    for (auto& pair : m_filterConfigurationsByName)
    {
        if (pair.second.defaultState() == FilterConfiguration::DefaultState::ENABLED)
        {
            m_filterPool->enable(strategyType(), pair.first, pair.second);
        }
    }
}

void BaseStrategy::onDisabling()
{
    for (auto& pair : m_filterConfigurationsByName)
    {
        m_filterPool->disable(strategyType(), pair.first);
    }
}
