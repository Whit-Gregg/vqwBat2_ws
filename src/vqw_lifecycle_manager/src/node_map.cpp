#include "vqw_lifecycle_manager/node_map.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>


// yaml file format:
// containers:
//   container_name_1:
//     - node_name: node1
//       target_state: active
//     - node_name: node2   
//       target_state: inactive             

bool ContainerMap::LoadFromYAML(const std::string &yaml_file_path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        if (!config["containers"])
        {
            RCLCPP_ERROR(rclcpp::get_logger("ContainerMap"), "YAML file does not contain 'containers' key");
            return false;
        }

        for (const auto &container_it : config["containers"])
        {
            std::string container_name = container_it.first.as<std::string>();
            auto container = std::make_shared<ComponentContainer>(container_name);

            for (const auto &node_it : container_it.second)
            {
                std::string node_name = node_it["node_name"].as<std::string>();
                std::string target_state_str = node_it["target_state"].as<std::string>();
                LifecycleState target_state = LifecycleState::UNKNOWN;

                if (target_state_str == "unconfigured")
                    target_state = LifecycleState::UNCONFIGURED;
                else if (target_state_str == "inactive")
                    target_state = LifecycleState::INACTIVE;
                else if (target_state_str == "active")
                    target_state = LifecycleState::ACTIVE;
                else if (target_state_str == "finalized")
                    target_state = LifecycleState::FINALIZED;

                auto lifecycle_node = std::make_shared<LifecycleNode>(node_name, container_name, target_state);
                container->add_node(lifecycle_node);
            }

            add_container(container);
        }
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ContainerMap"), "Failed to load YAML file: %s", e.what());
        return false;
    }

    return true;
}

std::string ContainerMap::to_string() const
{
    std::string result;
    for (const auto &container_pair : containers_)
    {
        result += "Container: " + container_pair.first + "\n";
        auto container = container_pair.second;
        for (const auto &node_pair : container->get_nodes())
        {
            auto node = node_pair.second;
            result += "  Node: " + node->get_name() + ", Target State: ";
            switch (node->get_target_state())
            {
                case LifecycleState::UNCONFIGURED:
                    result += "UNCONFIGURED";
                    break;
                case LifecycleState::INACTIVE:
                    result += "INACTIVE";
                    break;
                case LifecycleState::ACTIVE:
                    result += "ACTIVE";
                    break;
                case LifecycleState::FINALIZED:
                    result += "FINALIZED";
                    break;
                default:
                    result += "UNKNOWN";
                    break;
            }
            result += "\n";
        }
    }
    return result;
}