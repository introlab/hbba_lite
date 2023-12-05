#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <unordered_set>
#include <optional>

using namespace std;

struct Topic
{
    string name;
    int priority;
    ros::Duration timeout;
};

class ArbitrationNode
{
    ros::NodeHandle& m_nodeHandle;
    vector<ros::Subscriber> m_subscribers;
    ros::Publisher m_publisher;
    bool m_hasAdvertised;

    vector<Topic> m_topics;
    bool m_latch;

    optional<int> m_currentTopicIndex;
    ros::Time m_lastMessageTime;

public:
    ArbitrationNode(ros::NodeHandle& nodeHandle, vector<Topic> topics, bool latch)
        : m_nodeHandle(nodeHandle),
          m_hasAdvertised(false),
          m_topics(move(topics)),
          m_latch(latch),
          m_lastMessageTime(ros::Time::now())
    {
        for (size_t i = 0; i < m_topics.size(); i++)
        {
            m_subscribers.emplace_back(m_nodeHandle.subscribe<topic_tools::ShapeShifter>(
                m_topics[i].name,
                1,
                [this, i](const ros::MessageEvent<topic_tools::ShapeShifter>& msgEvent) { callback(i, msgEvent); }));
        }
    }

    void run() { ros::spin(); }

private:
    void callback(size_t i, const ros::MessageEvent<topic_tools::ShapeShifter>& msgEvent)
    {
        auto& msg = msgEvent.getConstMessage();
        if (!m_hasAdvertised)
        {
            advertise(msg);
        }

        if (m_currentTopicIndex == nullopt || m_topics[i].priority <= m_topics[*m_currentTopicIndex].priority ||
            (ros::Time::now() - m_lastMessageTime) > m_topics[*m_currentTopicIndex].timeout)
        {
            m_currentTopicIndex = i;
            m_lastMessageTime = ros::Time::now();
            m_publisher.publish(msg);
        }
    }

    void advertise(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
    {
        m_publisher = msg->advertise(m_nodeHandle, "out", 10, m_latch);
        m_hasAdvertised = true;
    }
};


bool getTopics(ros::NodeHandle& privateNodeHandle, vector<Topic>& topics)
{
    vector<string> value;

    XmlRpc::XmlRpcValue xmlTopics;
    privateNodeHandle.getParam("topics", xmlTopics);
    if (xmlTopics.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Invalid topics format");
        return false;
    }

    for (size_t i = 0; i < xmlTopics.size(); i++)
    {
        if (xmlTopics[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            xmlTopics[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString ||
            xmlTopics[i]["priority"].getType() != XmlRpc::XmlRpcValue::TypeInt ||
            xmlTopics[i]["timeout_s"].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            ROS_ERROR_STREAM(
                "Invalid topics["
                << i << "]: name must be a string, priority must be a int and timeout_s must be a double.");
            return false;
        }

        topics.emplace_back(Topic{
            static_cast<string>(xmlTopics[i]["name"]),
            static_cast<int>(xmlTopics[i]["priority"]),
            ros::Duration(static_cast<double>(xmlTopics[i]["timeout_s"]))});
    }

    return topics.size() > 0;
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "arbitration_node");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");

    vector<Topic> topics;
    if (!getTopics(privateNodeHandle, topics))
    {
        ROS_ERROR("The parameter topics must be set, not empty and valid.");
        return -1;
    }
    if (!hasUniquePriority(topics))
    {
        ROS_ERROR("The topic priorities must be unique.");
        return -1;
    }

    bool latch;
    if (!privateNodeHandle.getParam("latch", latch))
    {
        ROS_ERROR("The parameter latch is required.");
        return -1;
    }

    ArbitrationNode node(nodeHandle, topics, latch);
    node.run();

    return 0;
}
