#include <rclcpp/rclcpp.hpp>
// // // // #include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "bq25820_node/bq25820_node.hpp"

namespace bq25820_node
{

    void BQ25820Node::timer_callback()
    {
        charger_.spinOnce();

        publish_battery_state();
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

        if (battery_state_pub_) { battery_state_pub_->publish(bat_state_msg); }
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

    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
    void BQ25820Node::do_configure()
    {
        // Declare parameters with defaults
        this->declare_parameter<std::string>("device_path", "/dev/i2c-1");
        this->declare_parameter<double>("publish_rate", 1.0);

        // Get parameters
        device_path_  = this->get_parameter("device_path").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();

        // Initialize charger
        if (!charger_.initialize(device_path_)) { RCLCPP_ERROR(this->get_logger(), "Failed to initialize BQ25820 on %s", device_path_.c_str()); }
        else
            {
                // Create lifecycle publisher (not activated yet)
                battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::QoS(10));

                RCLCPP_INFO(this->get_logger(), "Configured BQ25820 node. Device: %s, publish_rate: %.2f Hz", device_path_.c_str(), publish_rate_);
            }
    }
    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
    void BQ25820Node::do_activate()
    {
        // Start timer
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / std::max(0.001, publish_rate_)), std::bind(&BQ25820Node::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "BQ25820 node activated");
    }
    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+
    //~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+

    // // // // //     // Lifecycle callbacks
    // // // // //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BQ25820Node::on_configure(const rclcpp_lifecycle::State & /*state*/)
    // // // // //     {
    // // // // //         // Declare parameters with defaults
    // // // // //         this->declare_parameter<std::string>("device_path", "/dev/i2c-1");
    // // // // //         this->declare_parameter<double>("publish_rate", 1.0);

    // // // // //         // Get parameters
    // // // // //         device_path_  = this->get_parameter("device_path").as_string();
    // // // // //         publish_rate_ = this->get_parameter("publish_rate").as_double();

    // // // // //         // Initialize charger
    // // // // //         if (!charger_.initialize(device_path_))
    // // // // //             {
    // // // // //                 RCLCPP_ERROR(this->get_logger(), "Failed to initialize BQ25820 on %s", device_path_.c_str());
    // // // // //                 return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    // // // // //             }

    // // // // //         // Create lifecycle publisher (not activated yet)
    // // // // //         battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::QoS(10));

    // // // // //         RCLCPP_INFO(this->get_logger(), "Configured BQ25820 node. Device: %s, publish_rate: %.2f Hz", device_path_.c_str(), publish_rate_);
    // // // // //         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    // // // // //     }

    // // // // //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BQ25820Node::on_activate(const rclcpp_lifecycle::State & /*state*/)
    // // // // //     {
    // // // // //         // Activate publisher
    // // // // //         if (battery_state_pub_) { battery_state_pub_->on_activate(); }

    // // // // //         // Start timer
    // // // // //         timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / std::max(0.001, publish_rate_)), std::bind(&BQ25820Node::timer_callback, this));

    // // // // //         RCLCPP_INFO(this->get_logger(), "BQ25820 node activated");
    // // // // //         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    // // // // //     }

    // // // // //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BQ25820Node::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
    // // // // //     {
    // // // // //         // Stop timer
    // // // // //         timer_.reset();

    // // // // //         // Deactivate publisher
    // // // // //         if (battery_state_pub_) { battery_state_pub_->on_deactivate(); }

    // // // // //         RCLCPP_INFO(this->get_logger(), "BQ25820 node deactivated");
    // // // // //         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    // // // // //     }

    // // // // //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BQ25820Node::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
    // // // // //     {
    // // // // //         timer_.reset();
    // // // // //         battery_state_pub_.reset();
    // // // // //         RCLCPP_INFO(this->get_logger(), "BQ25820 node cleaned up");
    // // // // //         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    // // // // //     }

    // // // // //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BQ25820Node::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
    // // // // //     {
    // // // // //         timer_.reset();
    // // // // //         if (battery_state_pub_ && battery_state_pub_->is_activated()) { battery_state_pub_->on_deactivate(); }
    // // // // //         battery_state_pub_.reset();
    // // // // //         RCLCPP_INFO(this->get_logger(), "BQ25820 node shutdown");
    // // // // //         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    // // // // //     }

    // // // // //     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BQ25820Node::on_error(const rclcpp_lifecycle::State & /*state*/)
    // // // // //     {
    // // // // //         RCLCPP_ERROR(this->get_logger(), "BQ25820 node entered error state");
    // // // // //         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    // // // // //     }

}       // namespace bq25820_node

// // // // // #include <rclcpp_components/register_node_macro.hpp>
// // // // // RCLCPP_COMPONENTS_REGISTER_NODE(bq25820_node::BQ25820Node)

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<bq25820_node::BQ25820Node>(options);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}