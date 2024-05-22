#include <rclcpp/rclcpp.hpp>
#include <rosbag2_generic_topic/rosbag2_node.hpp>

#include <unordered_set>
#include <optional>

using namespace std;

constexpr const char* NODE_NAME = "arbitration_node";

struct Topic
{
    string name;
    int priority;
    rclcpp::Duration timeout;
};

class ArbitrationNode : public rosbag2_generic_topic::Rosbag2Node
{
    rclcpp::TimerBase::SharedPtr m_initTimer;
    vector<shared_ptr<rosbag2_generic_topic::GenericSubscription>> m_subscribers;
    shared_ptr<rosbag2_generic_topic::GenericPublisher> m_publisher;

    vector<Topic> m_topics;

    optional<int> m_currentTopicIndex;
    rclcpp::Time m_lastMessageTime;

public:
    ArbitrationNode()
        : rosbag2_generic_topic::Rosbag2Node(NODE_NAME),
          m_lastMessageTime(get_clock()->now())
    {
        m_topics = convertToTopics(
            declare_parameter("topics", vector<string>{}),
            declare_parameter("priorities", vector<int64_t>{}),
            declare_parameter("timeout_s", vector<double>{}));
        if (!hasUniquePriority(m_topics))
        {
            throw std::runtime_error("The topic priorities must be unique.");
        }

        m_initTimer = create_wall_timer(std::chrono::seconds(1), std::bind(&ArbitrationNode::initTimerCallback, this));
    }

    void run()
    {
        rclcpp::spin(shared_from_this());
    }

private:
    vector<Topic> convertToTopics(const vector<string>& topics, const vector<int64_t>& priorities, const vector<double>& timeoutS)
    {
        if (topics.size() != priorities.size() || topics.size() != timeoutS.size())
        {
            throw std::runtime_error("The topics, priorities, timeout_s parameters must have the same size.");
        }

        vector<Topic> convertedTopics;
        for (size_t i = 0; i < topics.size(); i++)
        {
            auto expandedTopic = rclcpp::expand_topic_or_service_name(topics[i], get_name(), get_namespace(), false);
            auto timeout = chrono::duration<double, std::ratio<1>>(timeoutS[i]);
            convertedTopics.emplace_back(Topic{expandedTopic, static_cast<int>(priorities[i]), timeout});
        }

        return convertedTopics;
    }

    bool hasUniquePriority(const vector<Topic>& topics)
    {
        unordered_set<int> priorities;

        for (auto& topic : topics)
        {
            if (priorities.count(topic.priority) > 0)
            {
                return false;
            }

            priorities.insert(topic.priority);
        }

        return true;
    }

    void initTimerCallback()
    {
        auto topicType = getTopicType();
        if (topicType == nullopt)
        {
            return;
        }

        for (size_t i = 0; i < m_topics.size(); i++)
        {
            m_subscribers.emplace_back(create_generic_subscription(
                m_topics[i].name,
                *topicType,
                1,
                [this, i](shared_ptr<rclcpp::SerializedMessage> msg) { messageCallback(i, msg); }));
        }
        m_publisher = create_generic_publisher("out", *topicType, 10);

        m_initTimer->cancel();
    }

    optional<string> getTopicType()
    {
        auto all_topics_and_types = get_topic_names_and_types();
        string type;
        for (auto t : m_topics)
        {
            auto it = all_topics_and_types.find(t.name);
            if (it != all_topics_and_types.end() && !it->second.empty())
            {
                return it->second[0];
            }
        }

        return nullopt;
    }

    void messageCallback(size_t i, shared_ptr<rclcpp::SerializedMessage> msg)
    {
        if (m_currentTopicIndex == nullopt || m_topics[i].priority <= m_topics[*m_currentTopicIndex].priority ||
            (get_clock()->now() - m_lastMessageTime) > m_topics[*m_currentTopicIndex].timeout)
        {
            m_currentTopicIndex = i;
            m_lastMessageTime = get_clock()->now();
            m_publisher->publish(msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ArbitrationNode>();
        node->run();
        rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "%s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    return 0;
}
