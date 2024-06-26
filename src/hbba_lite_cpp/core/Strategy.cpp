#include <hbba_lite/core/Strategy.h>

#include <hbba_lite/utils/HbbaLiteException.h>

#include <limits>

using namespace std;

StrategyType::StrategyType(type_index type) : m_type(type) {}


FilterConfiguration::FilterConfiguration() : m_type(FilterType::ON_OFF), m_rate(0) {}

FilterConfiguration::FilterConfiguration(uint16_t rate) : m_type(FilterType::THROTTLING), m_rate(rate)
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

void FilterPool::enable(const string& name, const FilterConfiguration& configuration)
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

    if (it->second == 0)
    {
        applyEnabling(name, configuration);
        m_lastFilterConfigurationByName[name] = configuration;
    }
    else if (configuration != m_lastFilterConfigurationByName[name])
    {
        throw HbbaLiteException("Not compatible filter configuration (" + name + ")");
    }

    it->second++;
}

void FilterPool::disable(const string& name)
{
    lock_guard<recursive_mutex> lock(m_mutex);

    auto it = m_countsByName.find(name);
    if (it == m_countsByName.end())
    {
        throw HbbaLiteException("Not existing filter (" + name + ")");
    }

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
        m_filterPool->enable(pair.first, pair.second);
    }
}

void BaseStrategy::onDisabling()
{
    for (auto& pair : m_filterConfigurationsByName)
    {
        m_filterPool->disable(pair.first);
    }
}
