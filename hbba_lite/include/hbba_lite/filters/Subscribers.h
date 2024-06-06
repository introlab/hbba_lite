#ifndef HBBA_LITE_FILTERS_SUBSCRIBER_H
#define HBBA_LITE_FILTERS_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>

#include <hbba_lite/filters/FilterState.h>

template<class FilterState, class MessageType>
class HbbaSubscriber
{
    FilterState m_filterState;
    std::function<void(const typename MessageType::SharedPtr)> m_userCallback;
    typename rclcpp::Subscription<MessageType>::SharedPtr m_subscriber;

public:
    HbbaSubscriber(
        std::shared_ptr<rclcpp::Node>& node,
        const std::string& topic,
        uint32_t queueSize,
        std::function<void(const typename MessageType::SharedPtr)> userCallback,
        const std::string& stateServiceName = "");

    std::string getTopic() const;

    bool isFilteringAllMessages() const;

private:
    void callback(const typename MessageType::SharedPtr msg);
};

template<class FilterState, class MessageType>
HbbaSubscriber<FilterState, MessageType>::HbbaSubscriber(
    std::shared_ptr<rclcpp::Node>& node,
    const std::string& topic,
    uint32_t queueSize,
    std::function<void(const typename MessageType::SharedPtr)> userCallback,
    const std::string& stateServiceName)
    : m_filterState(node, stateServiceName == "" ? topic + "/filter_state" : stateServiceName),
      m_userCallback(std::move(userCallback))
{
    m_subscriber = node->create_subscription<MessageType>(
        topic,
        queueSize,
        [this](const typename MessageType::SharedPtr msg) { callback(msg); });
}

template<class FilterState, class MessageType>
std::string HbbaSubscriber<FilterState, MessageType>::getTopic() const
{
    return m_subscriber->get_topic_name();
}
template<class FilterState, class MessageType>
bool HbbaSubscriber<FilterState, MessageType>::isFilteringAllMessages() const
{
    return m_filterState.isFilteringAllMessages();
}

template<class FilterState, class MessageType>
void HbbaSubscriber<FilterState, MessageType>::callback(const typename MessageType::SharedPtr msg)
{
    if (m_filterState.check() && m_userCallback)
    {
        m_userCallback(msg);
    }
}

template<class MessageType>
using OnOffHbbaSubscriber = HbbaSubscriber<OnOffHbbaFilterState, MessageType>;

template<class MessageType>
using ThrottlingHbbaSubscriber = HbbaSubscriber<ThrottlingHbbaFilterState, MessageType>;

#endif
