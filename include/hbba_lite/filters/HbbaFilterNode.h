#ifndef HBBA_FILTER_FILTERS_NODE_H
#define HBBA_FILTER_FILTERS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_generic_topic/rosbag2_node.hpp>

#include <hbba_lite/filters/FilterState.h>

template<class FilterState>
class HbbaFilterNode : public rosbag2_generic_topic::Rosbag2Node
{
    std::unique_ptr<FilterState> m_filterState;

    std::string m_inputTopic;
    std::string m_outputTopic;
    std::string m_stateService;
    rclcpp::TimerBase::SharedPtr m_initTimer;

    std::shared_ptr<rosbag2_generic_topic::GenericSubscription> m_subscriber;
    std::shared_ptr<rosbag2_generic_topic::GenericPublisher> m_publisher;

public:
    HbbaFilterNode(const std::string& nodeName);
    void run();

private:
    void initTimerCallback();
    void callback(std::shared_ptr<rclcpp::SerializedMessage>);
};

template<class FilterState>
inline HbbaFilterNode<FilterState>::HbbaFilterNode(const std::string& nodeName)
    : rosbag2_generic_topic::Rosbag2Node(nodeName)
{
    m_inputTopic = rclcpp::expand_topic_or_service_name(declare_parameter("input_topic", "in"), get_name(), get_namespace(), false);
    m_outputTopic = rclcpp::expand_topic_or_service_name(declare_parameter("output_topic", "out"), get_name(), get_namespace(), false);
    m_stateService = rclcpp::expand_topic_or_service_name(declare_parameter("state_service", "filter_state"), get_name(), get_namespace(), true);

    m_initTimer = create_wall_timer(std::chrono::seconds(1), std::bind(&HbbaFilterNode<FilterState>::initTimerCallback, this));
}

template<class FilterState>
inline void HbbaFilterNode<FilterState>::run()
{
    rclcpp::spin(shared_from_this());
}

template<class FilterState>
void HbbaFilterNode<FilterState>::initTimerCallback()
{
    auto all_topics_and_types = get_topic_names_and_types();
    auto it = all_topics_and_types.find(m_inputTopic);
    if (it == all_topics_and_types.end() || it->second.empty())
    {
        return;
    }

    m_filterState = std::make_unique<FilterState>(shared_from_this(), m_stateService);

    m_subscriber = create_generic_subscription(m_inputTopic, it->second[0], 10,
        [this](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
            if (m_filterState->check())
            {
                m_publisher->publish(msg);
            }
        });
    m_publisher = create_generic_publisher(m_outputTopic, it->second[0], 10);

    m_initTimer->cancel();
}

typedef HbbaFilterNode<OnOffHbbaFilterState> OnOffHbbaFilterNode;
typedef HbbaFilterNode<ThrottlingHbbaFilterState> ThrottlingHbbaFilterNode;

#endif
