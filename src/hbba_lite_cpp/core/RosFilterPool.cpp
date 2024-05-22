#include <hbba_lite/core/RosFilterPool.h>
#include <hbba_lite/utils/HbbaLiteException.h>

#include <hbba_lite/srv/set_on_off_filter_state.hpp>
#include <hbba_lite/srv/set_throttling_filter_state.hpp>

using namespace std;

RosFilterPool::RosFilterPool(shared_ptr<rclcpp::Node> node, bool waitForService)
    : m_node(move(node)),
      m_waitForService(waitForService)
{
}

void RosFilterPool::add(const string& name, FilterType type)
{
    lock_guard<recursive_mutex> lock(m_mutex);
    FilterPool::add(name, type);

    switch (type)
    {
        case FilterType::ON_OFF:
            m_serviceClientsByName[name] = m_node->create_client<hbba_lite::srv::SetOnOffFilterState>(name);
            break;

        case FilterType::THROTTLING:
            m_serviceClientsByName[name] = m_node->create_client<hbba_lite::srv::SetThrottlingFilterState>(name);
            break;

        default:
            throw HbbaLiteException("Not supported filter type");
    }
}

void RosFilterPool::applyEnabling(const string& name, const FilterConfiguration& configuration)
{
    switch (m_typesByName[name])
    {
        case FilterType::ON_OFF:
        {
            auto request = make_shared<hbba_lite::srv::SetOnOffFilterState::Request>();
            request->is_filtering_all_messages = false;
            call<hbba_lite::srv::SetOnOffFilterState>(name, request);
        }
        break;

        case FilterType::THROTTLING:
        {
            auto request = make_shared<hbba_lite::srv::SetThrottlingFilterState::Request>();
            request->is_filtering_all_messages = false;
            request->rate = configuration.rate();
            call<hbba_lite::srv::SetThrottlingFilterState>(name, request);
        }
        break;

        default:
            throw HbbaLiteException("Not supported filter type");
    }
}

void RosFilterPool::applyDisabling(const string& name)
{
    switch (m_typesByName[name])
    {
        case FilterType::ON_OFF:
        {
            auto request = make_shared<hbba_lite::srv::SetOnOffFilterState::Request>();
            request->is_filtering_all_messages = true;
            call<hbba_lite::srv::SetOnOffFilterState>(name, request);
        }
        break;

        case FilterType::THROTTLING:
        {
            auto request = make_shared<hbba_lite::srv::SetThrottlingFilterState::Request>();
            request->is_filtering_all_messages = true;
            request->rate = 1;
            call<hbba_lite::srv::SetThrottlingFilterState>(name, request);
        }
        break;

        default:
            throw HbbaLiteException("Not supported filter type");
    }
}

RosLogFilterPoolDecorator::RosLogFilterPoolDecorator(shared_ptr<rclcpp::Node> node, unique_ptr<FilterPool> filterPool)
    : m_node(move(node)),
      m_filterPool(move(filterPool))
{
}

void RosLogFilterPoolDecorator::add(const string& name, FilterType type)
{
    lock_guard<recursive_mutex> lock(m_mutex);
    FilterPool::add(name, type);
    m_filterPool->add(name, type);
}

void RosLogFilterPoolDecorator::applyEnabling(const string& name, const FilterConfiguration& configuration)
{
    callApplyEnabling(*m_filterPool, name, configuration);

    switch (configuration.type())
    {
        case FilterType::ON_OFF:
            RCLCPP_INFO_STREAM(m_node->get_logger(), "HBBA filter state changed: " << name << " -> enabled");
            break;

        case FilterType::THROTTLING:
            RCLCPP_INFO_STREAM(m_node->get_logger(), "HBBA filter state changed: " << name << " -> enabled (" << configuration.rate() << ")");
            break;

        default:
            throw HbbaLiteException("Not supported filter type");
    }
}

void RosLogFilterPoolDecorator::applyDisabling(const std::string& name)
{
    callApplyDisabling(*m_filterPool, name);
    RCLCPP_INFO_STREAM(m_node->get_logger(), "HBBA filter state changed: " << name << " -> disabled");
}
