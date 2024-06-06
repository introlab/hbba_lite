#ifndef HBBA_LITE_FILTERS_PUBLISHERS_H
#define HBBA_LITE_FILTERS_PUBLISHERS_H

#include <rclcpp/rclcpp.hpp>

#include <hbba_lite/filters/FilterState.h>

template<class FilterState, class MessageType>
class HbbaPublisher
{
    FilterState m_filterState;
    typename rclcpp::Publisher<MessageType>::SharedPtr m_publisher;

public:
    HbbaPublisher(
        std::shared_ptr<rclcpp::Node>& node,
        const std::string& topic,
        uint32_t queueSize,
        const std::string& stateServiceName = "");

    uint32_t getNumSubscribers() const;
    std::string getTopic() const;

    bool isFilteringAllMessages() const;

    void publish(const MessageType& msg);
};

template<class FilterState, class MessageType>
inline HbbaPublisher<FilterState, MessageType>::HbbaPublisher(
    std::shared_ptr<rclcpp::Node>& node,
    const std::string& topic,
    uint32_t queueSize,
    const std::string& stateServiceName)
    : m_filterState(node, stateServiceName == "" ? topic + "/filter_state" : stateServiceName)
{
    m_publisher = node->create_publisher<MessageType>(topic, queueSize);
}

template<class FilterState, class MessageType>
inline uint32_t HbbaPublisher<FilterState, MessageType>::getNumSubscribers() const
{
    return m_publisher->get_subscription_count();
}

template<class FilterState, class MessageType>
std::string HbbaPublisher<FilterState, MessageType>::getTopic() const
{
    return m_publisher->get_topic_name();
}

template<class FilterState, class MessageType>
bool HbbaPublisher<FilterState, MessageType>::isFilteringAllMessages() const
{
    return m_filterState->isFilteringAllMessages();
}

template<class FilterState, class MessageType>
inline void HbbaPublisher<FilterState, MessageType>::publish(const MessageType& msg)
{
    if (m_filterState.check())
    {
        m_publisher->publish(msg);
    }
}

template<class MessageType>
using OnOffHbbaPublisher = HbbaPublisher<OnOffHbbaFilterState, MessageType>;

template<class MessageType>
using ThrottlingHbbaPublisher = HbbaPublisher<ThrottlingHbbaFilterState, MessageType>;

#endif
