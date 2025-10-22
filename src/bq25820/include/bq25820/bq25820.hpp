#ifndef BQ25820__BQ25820_HPP_
#define BQ25820__BQ25820_HPP_

#include "bq25820/bq25820_Registers.hpp"
#include "bq25820/bq25820_gpio.hpp"
#include "bq25820/bq25820_i2c.hpp"
#include "bq25820/elapsedMillis.h"
#include "bq25820/visibility_control.h"
#include <cstdint>
#include <limits>
#include <string>

namespace bq25820
{

    class Bq25820
    {
      public:
        Bq25820(const char *pDeviceName = "/dev/i2c-0", uint8_t i2c_address = 0x6B);       // Default I2C address for BBq25820 is 0x6B

        virtual ~Bq25820();

        bool initialize(std::string &device_name);
        bool spinOnce();       // return true if something changed       // call this periodically to handle background tasks, 2 or 3 Hz
        void shutdown();

        bool beginCharging();
        bool endCharging();

        const float max_volts_4S = 16.8F;
        const float min_volts_4S = 15.2F;

        float get_battery_voltage() const;
        float get_charging_current() const;
        float get_battery_percentage() const;
        bool  is_charging() const;

        void set_ChargeCurrentLimit(float limit_in_amps);
        void set_BatFET_is_ideal_diode(bool enable);
        void set_ChargeEnable(bool enable);
        void set_BatFET_Force_off(bool enable);
        void set_ADC_Control(bool enable, bool one_shot, bool average);

        std::string get_Status_str() const;
        std::string get_Status_changes_str() const;
        void        display_status_changes();

        bool update_statuses();       // true if changes in statuses

        uint8_t get_Charger_Status1() const { return i2c_.read_reg8(BQ25820_Register::REG0x21_Charger_Status_1); }
        uint8_t get_Charger_Status2() const { return i2c_.read_reg8(BQ25820_Register::REG0x22_Charger_Status_2); }
        uint8_t get_Charger_Status3() const { return i2c_.read_reg8(BQ25820_Register::REG0x23_Charger_Status_3); }
        uint8_t get_Fault_Status() const { return i2c_.read_reg8(BQ25820_Register::REG0x24_Fault_Status); }
        uint8_t get_Charger_Flag1() const { return i2c_.read_reg8(BQ25820_Register::REG0x25_Charger_Flag_1); }
        uint8_t get_Charger_Flag2() const { return i2c_.read_reg8(BQ25820_Register::REG0x26_Charger_Flag_2); }
        uint8_t get_Fault_Flag() const { return i2c_.read_reg8(BQ25820_Register::REG0x27_Fault_Flag); }
        uint8_t get_Part_Information() const { return i2c_.read_reg8(BQ25820_Register::REG0x3D_Part_Information); }

        std::string get_Charger_Status1_str(uint8_t stat1) const;
        std::string get_Charger_Status2_str(uint8_t stat2) const;
        std::string get_Charger_Status3_str(uint8_t stat3) const;
        std::string get_Fault_Status_str(uint8_t fault) const;
        std::string get_Charger_Flag1_str(uint8_t flag1) const;
        std::string get_Charger_Flag2_str(uint8_t flag2) const;
        std::string get_Fault_Flag_str(uint8_t fault_flag) const;
        std::string get_Part_Information_str(uint8_t part_info) const;

        std::string get_Charger_Status1_changes_str() const;
        std::string get_Charger_Status2_changes_str() const;
        std::string get_Charger_Status3_changes_str() const;
        std::string get_Fault_Status_changes_str() const;
        std::string get_Charger_Flag1_changes_str() const;
        std::string get_Charger_Flag2_changes_str() const;
        std::string get_Fault_Flag_changes_str() const;

        float get_ADC_IAC_value_amps() const { return ADC_IAC_value_amps_; }
        float get_ADC_IBAT_value_amps() const { return ADC_IBAT_value_amps_; }
        float get_ADC_VAC_value_volts() const { return ADC_VAC_value_volts_; }
        float get_ADC_VBAT_value_volts() const { return ADC_VBAT_value_volts_; }
        float get_ADC_VSYS_value_volts() const { return ADC_VSYS_value_volts_; }
        float get_ADC_TS_value_celsius() const { return ADC_TS_value_celsius_; }

        float get_battery_capacity() { return std::numeric_limits<double>::quiet_NaN(); }
        float get_battery_design_capacity() { return std::numeric_limits<double>::quiet_NaN(); }

        BQ25820_Charge_Status get_charging_status() const;
        BQ25820_Charge_Status charge_status_current_ {BQ25820_Charge_Status::Not_charging};
        BQ25820_Charge_Status charge_status_previous_ {BQ25820_Charge_Status::Not_charging};
        bool                  check_for_charging_status_change();

        std::string get_BQ25820_Charge_Status_str(BQ25820_Charge_Status status);

        std::string ADC_values_str();
        void        display_ADC_values();

        void check_gpio_changes();
        int gpio_INT_previous_ { -1 };
        int gpio_STAT1_previous_ { -1 };
        int gpio_STAT2_previous_ { -1 };
        int gpio_PG_previous_ { -1 };
        int gpio_CE_previous_ { -1 };

      private:
        // Private member variables and methods
        uint8_t     i2c_address_;
        std::string device_name_;
        BQ25820_I2C i2c_;
        //-------------------------------------------------------------------------------------
        BQ25820_GPIO gpio_;
        //-------------------------------------------------------------------------------------
        int get_gpio_INT() { return gpio_.get_gpio_10(); }
        int get_gpio_STAT1() { return gpio_.get_gpio_11(); }
        int get_gpio_STAT2() { return gpio_.get_gpio_12(); }
        int get_gpio_PG() { return gpio_.get_gpio_13(); }
        int get_gpio_CE() { return gpio_.get_gpio_16(); }
        //-------------------------------------------------------------------------------------
        //---- Status Data --------------------------------------------------------------
        uint8_t status1_current_ {0};
        uint8_t status2_current_ {0};
        uint8_t status3_current_ {0};
        uint8_t fault_status_current_ {0};
        uint8_t flag1_current_ {0};
        uint8_t flag2_current_ {0};
        uint8_t fault_flag_current_ {0};

        uint8_t status1_previous_ {0};
        uint8_t status2_previous_ {0};
        uint8_t status3_previous_ {0};
        uint8_t fault_status_previous_ {0};
        uint8_t flag1_previous_ {0};
        uint8_t flag2_previous_ {0};
        uint8_t fault_flag_previous_ {0};
        //-------------------------------------------------------------------------------------
        uint8_t        get_status_bits_changes(uint8_t old_stat1, uint8_t new_stat1) const { return old_stat1 ^ new_stat1; }
        elapsedMillis  elap_since_status_check;
        const uint32_t elap_since_status_check_interval = 5000;       // 5 seconds

        //-------------------------------------------------------------------------------------
        elapsedMillis  elap_since_Status_display_ {0};
        const uint32_t elap_since_Status_display_interval_ {30 * 1000};
        //-------------------------------------------------------------------------------------
        elapsedMillis  elap_since_ADC_read_ {0};
        const uint32_t elap_since_ADC_read_interval_ {5000};                                        // milliseconds between ADC reads
        const uint32_t elap_since_ADC_start_interval_ {elap_since_ADC_read_interval_ - 1500};       // 1.5 seconds before ADC read to start one-shot
        elapsedMillis  elap_since_last_ADC_display_ {0};
        const uint32_t elap_since_last_ADC_display_interval_ {30 * 1000};       // 30 seconds between ADC value displays
        // ADC control
        bool adc_OneShot_in_progress_ {false};
        //------------------------------------------------------------------------------------
        bool           is_charging_ {false};
        bool           should_start_charging() const;
        elapsedMillis  elap_since_charging_completed_ {0};
        const uint32_t elap_since_charging_completed_interval_ {60 * 1000};       // 1 minute
        //------------------------------------------------------------------------------------
        // ADC values
        float ADC_IAC_value_amps_ {0.0f};
        float ADC_IBAT_value_amps_ {0.0f};
        float ADC_VAC_value_volts_ {0.0f};
        float ADC_VBAT_value_volts_ {0.0f};
        float ADC_VSYS_value_volts_ {0.0f};
        float ADC_TS_value_celsius_ {0.0f};
        float ADC_TS_value_percent_of_REGN_ {0.0f};
        // Private methods
        void start_ADC_Conversion();
        // // bool check_if_ADC_conversion_complete();
        void read_all_ADC_values();
        bool input_power_good() const;
        // --------------------------------------------------------
        bool spin_ADC();                 // return true if something changed
        bool spin_status_change();       // return true if something changed
    };

}       // namespace bq25820

#endif       // BQ25820__BQ25820_HPP_
