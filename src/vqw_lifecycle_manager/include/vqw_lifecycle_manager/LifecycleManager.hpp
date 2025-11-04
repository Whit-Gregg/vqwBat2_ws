#pragma once

#include "node_map.hpp"
#include <rclcpp/rclcpp.hpp>

namespace vqw_lifecycle_manager
{

    class LifecycleManager : public rclcpp::Node
    {
      public:
        explicit LifecycleManager(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

      private:
        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
        void                         timer_callback();

        bool wait_for_service(ComponentContainer &container);         // returns true if service is available

        // Node map
        NodeMap node_map_;

        // Service client
        void query_node_state(const std::string &node_name);
        void set_node_state(const std::string &node_name, LifecycleState target_state);
    };

}       // namespace vqw_lifecycle_manager