#ifndef HBBA_LITE_CORE_STRATEGY_H
#define HBBA_LITE_CORE_STRATEGY_H

#include <hbba_lite/core/Desire.h>
#include <hbba_lite/utils/ClassMacros.h>

#include <cstdint>
#include <unordered_map>
#include <typeindex>
#include <string>
#include <mutex>
#include <memory>
#include <optional>

class BaseStrategy;

class StrategyType
{
    std::type_index m_type;

    explicit StrategyType(std::type_index type);

public:
    template<class T>
    static StrategyType get();

    const char* name() const;
};

template<class T>
inline StrategyType StrategyType::get()
{
    static_assert(std::is_base_of<BaseStrategy, T>::value, "T must be a subclass of BaseStrategy");
    return StrategyType(std::type_index(typeid(T)));
}

inline const char* StrategyType::name() const
{
    return m_type.name();
}


enum class FilterType
{
    ON_OFF,
    THROTTLING
};

inline std::string filterTypeToString(FilterType filterType)
{
    switch (filterType)
    {
        case FilterType::ON_OFF:
            return "ON_OFF";
        case FilterType::THROTTLING:
            return "THROTTLING";
        default:
            return "UNKNOWN";
    }
}

class FilterConfiguration
{
    FilterType m_type;
    uint16_t m_rate;

public:
    FilterConfiguration();
    explicit FilterConfiguration(uint16_t rate);

    FilterType type() const;
    uint16_t rate() const;

    static FilterConfiguration onOff();
    static FilterConfiguration throttling(uint16_t rate);

    friend bool operator==(const FilterConfiguration& a, const FilterConfiguration& b);
    friend bool operator!=(const FilterConfiguration& a, const FilterConfiguration& b);
};

inline FilterType FilterConfiguration::type() const
{
    return m_type;
}

inline uint16_t FilterConfiguration::rate() const
{
    return m_rate;
}

inline FilterConfiguration FilterConfiguration::onOff()
{
    return FilterConfiguration();
}

inline FilterConfiguration FilterConfiguration::throttling(uint16_t rate)
{
    return FilterConfiguration(rate);
}

inline bool operator==(const FilterConfiguration& a, const FilterConfiguration& b)
{
    if (a.m_type == FilterType::THROTTLING && b.m_type == FilterType::THROTTLING)
    {
        return a.m_rate == b.m_rate;
    }
    else
    {
        return a.m_type == b.m_type;
    }
}

inline bool operator!=(const FilterConfiguration& a, const FilterConfiguration& b)
{
    return !(a == b);
}

class FilterPool
{
    std::unordered_map<std::string, int> m_countsByName;

protected:
    std::unordered_map<std::string, FilterType> m_typesByName;
    std::unordered_map<std::string, FilterConfiguration> m_lastFilterConfigurationByName;

    std::recursive_mutex m_mutex;

public:
    FilterPool() = default;
    virtual ~FilterPool() = default;

    DECLARE_NOT_COPYABLE(FilterPool);
    DECLARE_NOT_MOVABLE(FilterPool);

    virtual void add(const std::string& name, FilterType type);
    void enable(const std::string& name, const FilterConfiguration& configuration);
    void disable(const std::string& name);

protected:
    virtual void applyEnabling(const std::string& name, const FilterConfiguration& configuration) = 0;
    virtual void applyDisabling(const std::string& name) = 0;

    // Useful for decorators
    void callApplyEnabling(FilterPool& filterPool, const std::string& name, const FilterConfiguration& configuration);
    void callApplyDisabling(FilterPool& filterPool, const std::string& name);
};

class BaseStrategy
{
    bool m_enabled;
    std::optional<uint64_t> m_desireId;

    uint16_t m_utility;
    std::unordered_map<std::string, uint16_t> m_resourcesByName;
    std::unordered_map<std::string, FilterConfiguration> m_filterConfigurationsByName;

protected:
    std::shared_ptr<FilterPool> m_filterPool;

public:
    BaseStrategy(
        uint16_t utility,
        std::unordered_map<std::string, uint16_t> resourcesByName,
        std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName,
        std::shared_ptr<FilterPool> filterPool);
    virtual ~BaseStrategy() = default;

    DECLARE_NOT_COPYABLE(BaseStrategy);
    DECLARE_NOT_MOVABLE(BaseStrategy);

    uint16_t utility() const;
    std::optional<uint64_t> desireId() const;

    void enable(const Desire& desire);
    void disable();
    bool enabled() const;

    const std::unordered_map<std::string, uint16_t>& resourcesByName() const;
    const std::unordered_map<std::string, FilterConfiguration> filterConfigurationsByName() const;

    virtual DesireType desireType() = 0;
    virtual StrategyType strategyType() = 0;

protected:
    virtual void onEnabling(const Desire& desire);
    virtual void onDisabling();
};

inline uint16_t BaseStrategy::utility() const
{
    return m_utility;
}

inline std::optional<uint64_t> BaseStrategy::desireId() const
{
    return m_desireId;
}

inline void BaseStrategy::enable(const Desire& desire)
{
    if (!m_enabled)
    {
        m_enabled = true;
        m_desireId = desire.id();
        onEnabling(desire);
    }
}

inline void BaseStrategy::disable()
{
    if (m_enabled)
    {
        m_enabled = false;
        m_desireId = std::nullopt;
        onDisabling();
    }
}

inline bool BaseStrategy::enabled() const
{
    return m_enabled;
}

inline const std::unordered_map<std::string, uint16_t>& BaseStrategy::resourcesByName() const
{
    return m_resourcesByName;
}

inline const std::unordered_map<std::string, FilterConfiguration> BaseStrategy::filterConfigurationsByName() const
{
    return m_filterConfigurationsByName;
}


template<class T>
class Strategy : public BaseStrategy
{
public:
    Strategy(
        uint16_t utility,
        std::unordered_map<std::string, uint16_t> resourcesByName,
        std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName,
        std::shared_ptr<FilterPool> filterPool);
    ~Strategy() override = default;

    DECLARE_NOT_COPYABLE(Strategy);
    DECLARE_NOT_MOVABLE(Strategy);

    DesireType desireType() override;
    StrategyType strategyType() override;

protected:
    void onEnabling(const Desire& desire) final;
    virtual void onEnabling(const T& desire);
};

template<class T>
Strategy<T>::Strategy(
    uint16_t utility,
    std::unordered_map<std::string, uint16_t> resourcesByName,
    std::unordered_map<std::string, FilterConfiguration> filterConfigurationByName,
    std::shared_ptr<FilterPool> filterPool)
    : BaseStrategy(utility, move(resourcesByName), move(filterConfigurationByName), move(filterPool))
{
}

template<class T>
inline DesireType Strategy<T>::desireType()
{
    return DesireType::get<T>();
}

template<class T>
inline StrategyType Strategy<T>::strategyType()
{
    return StrategyType::get<Strategy<T>>();
}

template<class T>
inline void Strategy<T>::onEnabling(const Desire& desire)
{
    BaseStrategy::onEnabling(desire);
    onEnabling(dynamic_cast<const T&>(desire));
}

template<class T>
inline void Strategy<T>::onEnabling(const T& desire)
{
}

#endif
