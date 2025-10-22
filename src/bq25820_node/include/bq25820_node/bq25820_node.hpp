#ifndef BQ25820_NODE_HPP
#define BQ25820_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "bq25820/bq25820.hpp"

/// @file bq25820_node.hpp
/// @brief ROS 2 node wrapper for the TI BQ25820 charger driver.

/// @brief ROS 2 node wrapping the TI BQ25820 charger driver.
///
/// Publishes sensor_msgs::msg::BatteryState at a configurable rate.
///
/// Parameters:
/// - device_path (string): I2C device path. Default: /dev/i2c-1
/// - publish_rate (double): Publish rate in Hz. Default: 1.0
class BQ25820Node : public rclcpp::Node
{
  public:
    /// @brief Construct the node and start the periodic publisher.
    BQ25820Node() : Node("bq25820_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("device_path", "/dev/i2c-1");
        this->declare_parameter<double>("publish_rate", 1.0);

        // Get parameters
        device_path_  = this->get_parameter("device_path").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();

        // Initialize charger
        charger_.initialize(device_path_);

        // Publishers
        battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
        // voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);
        // current_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_current", 10);
        // charging_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("charging_status", 10);

        // Timer for periodic updates
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_), std::bind(&BQ25820Node::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "BQ25820 Node initialized with device path: %s", device_path_.c_str());
    }

    
  private:
    bq25820::Bq25820 charger_;

    /// @brief Timer callback invoked at publish_rate_ to update and publish state.
    void timer_callback();

    /// @brief Build and publish a sensor_msgs::msg::BatteryState message from cached charger data.
    void publish_battery_state();

    /// @brief Initialize the charger hardware/driver using device_path_.
    /// @return true on success.
    bool initialize_charger();

    /// @brief Read all relevant charger registers into cached fields.
    void read_charger_data();

    /// @brief Read battery voltage in volts.
    /// @return Voltage (V).
    double read_battery_voltage();

    /// @brief Read charging/discharging current in amperes. Positive = charging, negative = discharging.
    /// @return Current (A).
    double read_charging_current();

    /// @brief Whether the pack is currently charging.
    /// @return true if charging.
    bool   is_charging();

    /// @brief Map internal status to sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS.
    uint8_t get_POWER_SUPPLY_STATUS();
    /// @brief Map internal health to sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH.
    uint8_t get_POWER_SUPPLY_HEALTH();
    /// @brief Map internal technology to sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY.
    uint8_t get_POWER_SUPPLY_TECHNOLOGY();

    float voltage_previous_;
    float temperature_previous_;
    float current_previous_;
    float   capacity_previous_;
    float   design_capacity_previous_;
    float   percentage_previous_;
    uint8_t power_supply_status_previous_;
    uint8_t power_supply_health_previous_;
    uint8_t power_supply_technology_previous_;
    bool    present_previous_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Charger data
    float battery_voltage_;
    float battery_current_;
    float battery_temperature_;
    bool  is_charging_;
    bool  is_connected_;

    // Parameters
    std::string device_path_;
    double      publish_rate_;
};

#endif       // BQ25820_NODE_HPP