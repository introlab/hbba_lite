#ifndef HBBA_LITE_CORE_ROS_STRATEGY_H
#define HBBA_LITE_CORE_ROS_STRATEGY_H

#include <hbba_lite/core/Strategy.h>

#include <rclcpp/rclcpp.hpp>

#include <unordered_map>

class RosFilterPool : public FilterPool
{
    std::shared_ptr<rclcpp::Node> m_node;
    bool m_waitForService;

    std::unordered_map<std::string, rclcpp::ClientBase::SharedPtr> m_serviceClientsByName;

public:
    RosFilterPool(std::shared_ptr<rclcpp::Node> node, bool waitForService);
    ~RosFilterPool() override = default;

    DECLARE_NOT_COPYABLE(RosFilterPool);
    DECLARE_NOT_MOVABLE(RosFilterPool);

    void add(const std::string& name, FilterType type) override;

protected:
    void applyEnabling(const std::string& name, const FilterConfiguration& configuration) override;
    void applyDisabling(const std::string& name) override;

private:
    template<class ServiceType>
    void call(const std::string& name, std::shared_ptr<typename ServiceType::Request> request);
};

template<class ServiceType>
void RosFilterPool::call(const std::string& name, std::shared_ptr<typename ServiceType::Request> request)
{
    auto client = std::dynamic_pointer_cast<rclcpp::Client<ServiceType>>(m_serviceClientsByName[name]);
    if (client == nullptr)
    {
        RCLCPP_ERROR(m_node->get_logger(), "The service type is not valid");
        return;
    }

    if (m_waitForService)
    {
        client->wait_for_service();
    }

    auto result = client->async_send_request(request);
    auto futureReturnCode = rclcpp::spin_until_future_complete(m_node, result);

    if (futureReturnCode != rclcpp::FutureReturnCode::SUCCESS || !result.get()->ok)
    {
        RCLCPP_ERROR(m_node->get_logger(), "The service call has failed (%s)", name.c_str());
    }
}

class RosLogFilterPoolDecorator : public FilterPool
{
    std::shared_ptr<rclcpp::Node> m_node;
    std::unique_ptr<FilterPool> m_filterPool;

public:
    RosLogFilterPoolDecorator(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<FilterPool> filterPool);
    ~RosLogFilterPoolDecorator() override = default;

    DECLARE_NOT_COPYABLE(RosLogFilterPoolDecorator);
    DECLARE_NOT_MOVABLE(RosLogFilterPoolDecorator);

    void add(const std::string& name, FilterType type) override;

protected:
    void applyEnabling(const std::string& name, const FilterConfiguration& configuration) override;
    void applyDisabling(const std::string& name) override;
};


#endif
