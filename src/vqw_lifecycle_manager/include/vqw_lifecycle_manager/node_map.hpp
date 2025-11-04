#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
//#include <yaml-cpp/yaml.h

// yaml file format:
//
// containers:
//   container_name_1:
//     - node_name: node1
//       target_state: active
//     - node_name: node2
//       target_state: inactive
//   container_name_2:
//     - node_name: node3
//       target_state: active

namespace vqw_lifecycle_manager
{

    enum class LifecycleState
    {
        UNKNOWN,
        UNCONFIGURED,
        INACTIVE,
        ACTIVE,
        FINALIZED
    };

    enum class action_pending
    {
        NO_PENDING,
        QUERY_STATE_PENDING,
        SET_STATE_PENDING
    };

    //~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^
    //~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^
    class LifecycleNode
    {
      public:
        LifecycleNode(const std::string &name, const std::string &container_name, LifecycleState target_state) : name_(name), container_name_(container_name), target_state_(target_state) {}

        using SharedPtr = std::shared_ptr<LifecycleNode>;

        std::string    get_name() const { return name_; }
        std::string    get_container_name() const { return container_name_; }
        LifecycleState get_target_state() const { return target_state_; }
        void           set_current_state(LifecycleState state) { current_state_ = state; }
        LifecycleState get_current_state() const { return current_state_; }

      private:
        std::string    name_;
        std::string container_name_;
        LifecycleState target_state_;
        LifecycleState current_state_ = LifecycleState::UNKNOWN;
        action_pending pending_action_ = action_pending::NO_PENDING;
    };

    //~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^
    //~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^
    class ComponentContainer
    {
      public:
        ComponentContainer(std::string container_name) : container_name_(container_name) {}

        using SharedPtr = std::shared_ptr<ComponentContainer>;

        void add_node(const LifecycleNode::SharedPtr &node) { nodes_[node->get_name()] = node; }

        LifecycleNode::SharedPtr get_node(const std::string &name)
        {
            auto it = nodes_.find(name);
            if (it != nodes_.end()) { return it->second; }
            return nullptr;
        }

        std::string get_container_name() const { return container_name_; }

        std::unordered_map<std::string, LifecycleNode::SharedPtr> &get_nodes() { return nodes_; }

      private:
        std::string container_name_;
        std::unordered_map<std::string, LifecycleNode::SharedPtr> nodes_;
    };

    //~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^
    //~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^~-+^
    class ContainerMap
    {
      public:
        ContainerMap() = default;
        using SharedPtr = std::shared_ptr<ContainerMap>;

        void add_container(const ComponentContainer::SharedPtr &container) { containers_[container->get_container_name()] = container; }

        ComponentContainer::SharedPtr get_container(const std::string &name)
        {
            auto it = containers_.find(name);
            if (it != containers_.end()) { return it->second; }
            return nullptr;
        }

        std::unordered_map<std::string, ComponentContainer::SharedPtr> &get_containers() { return containers_; }

        bool LoadFromYAML(const std::string &yaml_file_path);

        std::string to_string() const;

      private:
        std::unordered_map<std::string, ComponentContainer::SharedPtr> containers_;
    };

}       // namespace vqw_lifecycle_manager