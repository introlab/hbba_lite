#ifndef HBBA_LITE_FILTERS_FILTER_STATE_H
#define HBBA_LITE_FILTERS_FILTER_STATE_H

#include <rclcpp/rclcpp.hpp>
#include <hbba_lite_srvs/srv/set_on_off_filter_state.hpp>
#include <hbba_lite_srvs/srv/set_throttling_filter_state.hpp>

#include <string>

class OnOffHbbaFilterState
{
    rclcpp::Service<hbba_lite_srvs::srv::SetOnOffFilterState>::SharedPtr m_stateService;
    bool m_isFilteringAllMessages;

public:
    OnOffHbbaFilterState(const std::shared_ptr<rclcpp::Node>& node, const std::string& stateServiceName);
    OnOffHbbaFilterState(rclcpp::Node& node, const std::string& stateServiceName);
    bool check();
    bool isFilteringAllMessages() const;

private:
    void stateServiceCallback(
        const std::shared_ptr<hbba_lite_srvs::srv::SetOnOffFilterState::Request> request,
        std::shared_ptr<hbba_lite_srvs::srv::SetOnOffFilterState::Response> response);
};

inline bool OnOffHbbaFilterState::isFilteringAllMessages() const
{
    return m_isFilteringAllMessages;
}

class ThrottlingHbbaFilterState
{
    rclcpp::Service<hbba_lite_srvs::srv::SetThrottlingFilterState>::SharedPtr m_stateService;
    bool m_isFilteringAllMessages;
    int m_rate;
    int m_counter;

public:
    ThrottlingHbbaFilterState(const std::shared_ptr<rclcpp::Node>& node, const std::string& stateServiceName);
    ThrottlingHbbaFilterState(rclcpp::Node& node, const std::string& stateServiceName);
    bool check();
    bool isFilteringAllMessages() const;

private:
    void stateServiceCallback(
        const std::shared_ptr<hbba_lite_srvs::srv::SetThrottlingFilterState::Request> request,
        std::shared_ptr<hbba_lite_srvs::srv::SetThrottlingFilterState::Response> response);
};

inline bool ThrottlingHbbaFilterState::isFilteringAllMessages() const
{
    return m_isFilteringAllMessages;
}

#endif
