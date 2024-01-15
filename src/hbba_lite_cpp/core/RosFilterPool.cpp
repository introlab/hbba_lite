#include <hbba_lite/core/RosFilterPool.h>
#include <hbba_lite/utils/HbbaLiteException.h>

#include <hbba_lite/SetOnOffFilterState.h>
#include <hbba_lite/SetThrottlingFilterState.h>

using namespace std;

RosFilterPool::RosFilterPool(ros::NodeHandle& nodeHandle, bool waitForService)
    : m_nodeHandle(nodeHandle),
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
            m_serviceClientsByName[name] = m_nodeHandle.serviceClient<hbba_lite::SetOnOffFilterState>(name, true);
            break;

        case FilterType::THROTTLING:
            m_serviceClientsByName[name] = m_nodeHandle.serviceClient<hbba_lite::SetThrottlingFilterState>(name, true);
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
            hbba_lite::SetOnOffFilterState srv;
            srv.request.is_filtering_all_messages = false;
            call(name, srv);
        }
        break;

        case FilterType::THROTTLING:
        {
            hbba_lite::SetThrottlingFilterState srv;
            srv.request.is_filtering_all_messages = false;
            srv.request.rate = configuration.rate();
            call(name, srv);
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
            hbba_lite::SetOnOffFilterState srv;
            srv.request.is_filtering_all_messages = true;
            call(name, srv);
        }
        break;

        case FilterType::THROTTLING:
        {
            hbba_lite::SetThrottlingFilterState srv;
            srv.request.is_filtering_all_messages = true;
            srv.request.rate = 1;
            call(name, srv);
        }
        break;

        default:
            throw HbbaLiteException("Not supported filter type");
    }
}

RosLogFilterPoolDecorator::RosLogFilterPoolDecorator(unique_ptr<FilterPool> filterPool) : m_filterPool(move(filterPool))
{
}

void RosLogFilterPoolDecorator::add(const std::string& name, FilterType type)
{
    lock_guard<recursive_mutex> lock(m_mutex);
    FilterPool::add(name, type);
    m_filterPool->add(name, type);
}

void RosLogFilterPoolDecorator::applyEnabling(const std::string& name, const FilterConfiguration& configuration)
{
    callApplyEnabling(*m_filterPool, name, configuration);

    switch (configuration.type())
    {
        case FilterType::ON_OFF:
            ROS_INFO_STREAM("HBBA filter state changed: " << name << " -> enabled");
            break;

        case FilterType::THROTTLING:
            ROS_INFO_STREAM("HBBA filter state changed: " << name << " -> enabled (" << configuration.rate() << ")");
            break;

        default:
            throw HbbaLiteException("Not supported filter type");
    }
}

void RosLogFilterPoolDecorator::applyDisabling(const std::string& name)
{
    callApplyDisabling(*m_filterPool, name);
    ROS_INFO_STREAM("HBBA filter state changed: " << name << " -> disabled");
}
