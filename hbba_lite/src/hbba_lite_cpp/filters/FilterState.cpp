#include <hbba_lite/filters/FilterState.h>

using namespace std;

OnOffHbbaFilterState::OnOffHbbaFilterState(const shared_ptr<rclcpp::Node>& node, const string& stateServiceName)
    : OnOffHbbaFilterState(*node, stateServiceName)
{
}

OnOffHbbaFilterState::OnOffHbbaFilterState(rclcpp::Node& node, const string& stateServiceName)
    : m_isFilteringAllMessages(true)
{
    using namespace std::placeholders;
    m_stateService = node.create_service<hbba_lite_srvs::srv::SetOnOffFilterState>(
        stateServiceName,
        bind(&OnOffHbbaFilterState::stateServiceCallback, this, _1, _2));
}

bool OnOffHbbaFilterState::check()
{
    return !m_isFilteringAllMessages;
}

void OnOffHbbaFilterState::stateServiceCallback(
    const shared_ptr<hbba_lite_srvs::srv::SetOnOffFilterState::Request> request,
    shared_ptr<hbba_lite_srvs::srv::SetOnOffFilterState::Response> response)
{
    m_isFilteringAllMessages = request->is_filtering_all_messages;
    response->ok = true;
}

ThrottlingHbbaFilterState::ThrottlingHbbaFilterState(
    const shared_ptr<rclcpp::Node>& node,
    const string& stateServiceName)
    : ThrottlingHbbaFilterState(*node, stateServiceName)
{
}

ThrottlingHbbaFilterState::ThrottlingHbbaFilterState(
    rclcpp::Node& node,
    const string& stateServiceName)
    : m_isFilteringAllMessages(true),
      m_rate(1),
      m_counter(0)
{
    using namespace std::placeholders;
    m_stateService = node.create_service<hbba_lite_srvs::srv::SetThrottlingFilterState>(
        stateServiceName,
        bind(&ThrottlingHbbaFilterState::stateServiceCallback, this, _1, _2));
}

bool ThrottlingHbbaFilterState::check()
{
    if (m_isFilteringAllMessages)
    {
        return false;
    }

    bool isReady = false;
    if (m_counter == 0)
    {
        isReady = true;
    }
    m_counter = (m_counter + 1) % m_rate;
    return isReady;
}

void ThrottlingHbbaFilterState::stateServiceCallback(
    const shared_ptr<hbba_lite_srvs::srv::SetThrottlingFilterState::Request> request,
    shared_ptr<hbba_lite_srvs::srv::SetThrottlingFilterState::Response> response)
{
    if (request->rate <= 0)
    {
        response->ok = false;
    }
    else
    {
        m_isFilteringAllMessages = request->is_filtering_all_messages;
        m_rate = request->rate;
        m_counter = 0;

        response->ok = true;
    }
}
