#include "LifecycleManager.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
namespace vqw_lifecycle_manager
{
    LifecycleManager::LifecycleManager(const std::string &node_name, const rclcpp::NodeOptions &options) : rclcpp::Node(node_name, options)
    {
        // Start timer
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&LifecycleManager::timer_callback, this));

        RCLCPP_INFO(get_logger(), "LifecycleManager initialized...");
    }

    void LifecycleManager::timer_callback() { RCLCPP_INFO(get_logger(), "LifecycleManager timer callback triggered."); }

// TRANSITION_CONFIGURE
// TRANSITION_ACTIVATE
// TRANSITION_DEACTIVATE
// TRANSITION_CLEANUP


    bool LifecycleManager::wait_for_service(ComponentContainer &container)
    {

      // Optionally wait for the service to be available
    // while (!client_->wait_for_service(std::chrono::seconds(1))) {
    //   if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //     return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    //   }
    //   RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    // }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  
    }


    void LifecycleManager::set_node_state(const LifecycleNode &node, LifecycleState target_state)
    {
        // ... inside your manager node's class or function
        std::string service_name = "/" + node.get_name() + "/change_state";
        auto client            = this->create_client<lifecycle_msgs::srv::ChangeState>(service_name);
        auto request           = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        if (target_state == LifecycleState::CONFIGURE)
            request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        else if (target_state == LifecycleState::ACTIVATE)
            request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        else if (target_state == LifecycleState::DEACTIVATE)
            request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        else if (target_state == LifecycleState::CLEANUP)
            request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

        while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

        auto result = client->async_send_request(request);
        // Handle the result of the transition request

        // auto result = client->async_send_request(request,std::bind(&MyLifecycleNode::service_response_callback, this, std::placeholders::_1);
    }
    //~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=
    //~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=
    //~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=~^-+=
}       // namespace vqw_lifecycle_manager



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleManager>("lifecycle_manager");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
