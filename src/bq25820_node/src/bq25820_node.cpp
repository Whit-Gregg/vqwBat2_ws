#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "bq25820_node/bq25820_node.hpp"

// BQ25820Node::BQ25820Node()
// {
//     // Publishers
//     battery_voltage_pub_ = this->create_publisher<std_msgs::msg::Float64>("battery_voltage", 10);
//     charging_current_pub_ = this->create_publisher<std_msgs::msg::Float64>("charging_current", 10);
//     charging_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("charging_status", 10);

//     // Timer for periodic updates
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(1000),
//         std::bind(&BQ25820Node::timer_callback, this)
//     );

//     charger_.initialize();

//     RCLCPP_INFO(this->get_logger(), "BQ25820 Node initialized");
// }

void BQ25820Node::timer_callback()
{
    charger_.spinOnce();

    publish_battery_state();

    // // Read battery data (placeholder - implement actual I2C communication)
    // auto voltage_msg = std_msgs::msg::Float64();
    // auto current_msg = std_msgs::msg::Float64();
    // auto status_msg = std_msgs::msg::Bool();

    // voltage_msg.data = read_battery_voltage();
    // current_msg.data = read_charging_current();
    // status_msg.data = is_charging();

    // battery_voltage_pub_->publish(voltage_msg);
    // charging_current_pub_->publish(current_msg);
    // charging_status_pub_->publish(status_msg);
}

void BQ25820Node::publish_battery_state()
{
    float voltage     = charger_.get_battery_voltage();
    float temperature = charger_.get_ADC_TS_value_celsius();
    float current     = charger_.get_ADC_IBAT_value_amps();       // Negative if discharging !!!
    // float charge = charger_.???
    float capacity                = charger_.get_battery_capacity();
    float design_capacity         = charger_.get_battery_design_capacity();
    float percentage              = charger_.get_battery_percentage();
    auto  power_supply_status     = get_POWER_SUPPLY_STATUS();
    auto  power_supply_health     = get_POWER_SUPPLY_HEALTH();
    auto  power_supply_technology = get_POWER_SUPPLY_TECHNOLOGY();
    bool  present                 = voltage > 2.0F;

    bool has_changed = false;
    has_changed |= (voltage != voltage_previous_);
    has_changed |= (temperature != temperature_previous_);
    has_changed |= (current != current_previous_);
    // has_changed |= (capacity != capacity_previous_);
    // has_changed |= (design_capacity != design_capacity_previous_);
    has_changed |= (percentage != percentage_previous_);
    has_changed |= (power_supply_status != power_supply_status_previous_);
    has_changed |= (power_supply_health != power_supply_health_previous_);
    has_changed |= (power_supply_technology != power_supply_technology_previous_);
    has_changed |= (present != present_previous_);

    if (!has_changed) return;

    auto bat_state_msg = sensor_msgs::msg::BatteryState();

    bat_state_msg.set__voltage(voltage);
    bat_state_msg.set__temperature(temperature);
    bat_state_msg.set__current(current);
    bat_state_msg.set__capacity(capacity);
    bat_state_msg.set__design_capacity(design_capacity);
    bat_state_msg.set__percentage(percentage);
    bat_state_msg.set__power_supply_status(power_supply_status);
    bat_state_msg.set__power_supply_health(power_supply_health);
    bat_state_msg.set__power_supply_technology(power_supply_technology);
    bat_state_msg.set__present(present);

    voltage_previous_                 = voltage;
    temperature_previous_             = temperature;
    current_previous_                 = current;
    capacity_previous_                = capacity;
    design_capacity_previous_         = design_capacity;
    percentage_previous_              = percentage;
    power_supply_status_previous_     = power_supply_status;
    power_supply_health_previous_     = power_supply_health;
    power_supply_technology_previous_ = power_supply_technology;
    present_previous_                 = present;

    battery_state_pub_->publish(bat_state_msg);
}

uint8_t BQ25820Node::get_POWER_SUPPLY_STATUS()
{
    uint8_t result     = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    float   percentage = charger_.get_battery_percentage();
    if (percentage > 90.0F) result = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    if (is_charging()) result = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;

    return result;
}

uint8_t BQ25820Node::get_POWER_SUPPLY_HEALTH() { return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD; }

uint8_t BQ25820Node::get_POWER_SUPPLY_TECHNOLOGY() { return sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION; }

double BQ25820Node::read_battery_voltage() { return charger_.get_battery_voltage(); }

double BQ25820Node::read_charging_current() { return charger_.get_charging_current(); }

bool BQ25820Node::is_charging() { return charger_.is_charging(); }

// rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_pub_;
// rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr charging_current_pub_;
// rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charging_status_pub_;
// rclcpp::TimerBase::SharedPtr timer_;

//~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
//~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
//~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
//~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+

int main(int argc, char **argv)
{
    std::string node_name {"bq25820_node"};
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BQ25820Node>());
    rclcpp::shutdown();
    return 0;
}