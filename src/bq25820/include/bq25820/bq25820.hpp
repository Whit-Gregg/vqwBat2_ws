#ifndef BQ25820__BQ25820_HPP_
#define BQ25820__BQ25820_HPP_

/**
 * @file bq25820.hpp
 * @brief High-level driver interface for the TI BQ25820 battery charger.
 *
 * Provides a C++ class wrapper around I2C registers and GPIOs to manage
 * charging state, read ADC measurements, and report charger/fault/status
 * information for the BQ25820 device.
 */

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

    /**
     * @brief BQ25820 charger controller.
     *
     * Encapsulates initialization, periodic processing, charging control, and
     * access to status/ADC data for a TI BQ25820 charger IC via I2C and GPIOs.
     */
    class Bq25820
    {
      public:
        /**
         * @brief Construct a Bq25820 controller.
         * @param pDeviceName I2C device path (e.g., "/dev/i2c-1").
         * @param i2c_address 7-bit I2C address (default 0x6B).
         */
        Bq25820(const char *pDeviceName = "/dev/i2c-0", uint8_t i2c_address = 0x6B);       // Default I2C address for BBq25820 is 0x6B

        /// Destructor.
        virtual ~Bq25820();

        /**
         * @brief Initialize communication and configure device.
         * @param device_name Reference to a string to receive the effective device name.
         * @return true on success, false otherwise.
         */
        bool initialize(std::string &device_name);
        /**
         * @brief Perform periodic background tasks.
         * @return true if any state or measurement changed since last call.
         * @note Call at ~2–3 Hz.
         */
        bool spinOnce();       // return true if something changed       // call this periodically to handle background tasks, 2 or 3 Hz
        /// Shut down and release resources if needed.
        void shutdown();

        /// Begin charging sequence (enables charging per configuration).
        bool beginCharging();
        /// End charging sequence (disables charging).
        bool endCharging();

        int       spin_once_skip_count     = 0;
        const int max_spin_once_skip_count = 25;

        /// Maximum voltage for 4S configuration (V).
        const float max_volts_4S = 16.8F;
        /// Minimum voltage threshold for 4S configuration (V).
        const float min_volts_4S = 13.0F;

        /// Get the current battery voltage (V).
        float get_battery_voltage() const;
        /// Get the current charging current (A).
        float get_charging_current() const;
        /// Estimate the battery state of charge in percent [0..100].
        float get_battery_percentage() const;
        /// Return true if charger reports actively charging.
        bool is_charging();

        void          reset_watchdog_timer();
        elapsedMillis elap_since_watchdog_reset_;
        uint32_t      watchdog_reset_interval_ms_ = 30000;       // reset watchdog every 30 seconds

        /**
         * @brief Set charge current limit.
         * @param limit_in_amps Charge current in amperes.
         */
        void set_ChargeCurrentLimit(float limit_in_amps);

        void set_ChargeVoltageLimit(float limit_in_volts);  // min = 16.37V, max = 17.04V

        void disable_ICHG_pin_control(bool disable);       // disable ICHG pin control by setting EN_ICHG_PIN=0

        /// Configure BatFET as ideal diode when enabled.
        void set_BatFET_is_ideal_diode(bool enable);

        /// Enable or disable charging.
        void set_ChargeEnable(bool enable);

        /// Force BatFET off when enabled.
        void set_BatFET_Force_off(bool enable);

        /// Configure ADC control (enable, one-shot, averaging).
        void set_ADC_Control(bool enable, bool one_shot, bool average);

        /// Get a human-readable combined status string.
        std::string get_Status_str();
        /// Get a human-readable string of recent status changes.
        std::string get_Status_changes_str() const;
        /// Print or otherwise display status changes.
        void display_status_changes();

        /// Refresh cached status registers; return true if any bit changed.
        bool update_statuses();       // true if changes in statuses

        /// Raw register getters (see datasheet for bit definitions).
        uint8_t get_Charger_Status1() { return i2c_.read_reg8(BQ25820_Register::REG0x21_Charger_Status_1); }
        uint8_t get_Charger_Status2() { return i2c_.read_reg8(BQ25820_Register::REG0x22_Charger_Status_2); }
        uint8_t get_Charger_Status3() { return i2c_.read_reg8(BQ25820_Register::REG0x23_Charger_Status_3); }
        uint8_t get_Fault_Status() { return i2c_.read_reg8(BQ25820_Register::REG0x24_Fault_Status); }
        uint8_t get_Charger_Flag1() { return i2c_.read_reg8(BQ25820_Register::REG0x25_Charger_Flag_1); }
        uint8_t get_Charger_Flag2() { return i2c_.read_reg8(BQ25820_Register::REG0x26_Charger_Flag_2); }
        uint8_t get_Fault_Flag() { return i2c_.read_reg8(BQ25820_Register::REG0x27_Fault_Flag); }
        uint8_t get_Part_Information() { return i2c_.read_reg8(BQ25820_Register::REG0x3D_Part_Information); }

        /// Decode raw register values into descriptive strings.
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

        /// Latest ADC readings (units in name).
        float get_ADC_IAC_value_amps() const { return ADC_IAC_value_amps_; }
        float get_ADC_IBAT_value_amps() const { return ADC_IBAT_value_amps_; }
        float get_ADC_VAC_value_volts() const { return ADC_VAC_value_volts_; }
        float get_ADC_VBAT_value_volts() const { return ADC_VBAT_value_volts_; }
        float get_ADC_VSYS_value_volts() const { return ADC_VSYS_value_volts_; }
        float get_ADC_TS_value_celsius() const { return ADC_TS_value_celsius_; }

        /// Placeholder capacity values (currently not provided; returns NaN).
        float get_battery_capacity() { return std::numeric_limits<double>::quiet_NaN(); }
        float get_battery_design_capacity() { return std::numeric_limits<double>::quiet_NaN(); }

        /// Get the current charging status enumeration.
        BQ25820_Charge_Status get_charging_status();
        /// Current and previous charging status snapshots.
        BQ25820_Charge_Status charge_status_current_ {BQ25820_Charge_Status::Not_charging};
        BQ25820_Charge_Status charge_status_previous_ {BQ25820_Charge_Status::Not_charging};
        /// Check for a change in charging status since last snapshot.
        bool check_for_charging_status_change();

        /// Convert a charge status enum to a descriptive string.
        std::string get_BQ25820_Charge_Status_str(BQ25820_Charge_Status status);

        /// Format ADC values as a single string; optionally display them.
        std::string ADC_values_str();
        void        display_ADC_values();

        /// Poll GPIOs and update edge/change detection state.
        bool skip_gpio = false;
        void check_gpio_changes();
        int  gpio_INT_previous_ {-1};
        int  gpio_STAT1_previous_ {-1};
        int  gpio_STAT2_previous_ {-1};
        int  gpio_PG_previous_ {-1};
        int  gpio_CE_previous_ {-1};

      private:
        // Private member variables and methods
        /// 7-bit I2C address of the device.
        uint8_t i2c_address_;
        /// Selected I2C device path.
        std::string device_name_;
        /// I2C access helper.
        BQ25820_I2C i2c_;
        //-------------------------------------------------------------------------------------
        /// GPIO access helper.
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
        /// XOR helper for change detection between two register snapshots.
        uint8_t get_status_bits_changes(uint8_t old_stat1, uint8_t new_stat1) const { return old_stat1 ^ new_stat1; }
        /// Wall-clock since last status poll; and the poll interval (ms).
        elapsedMillis  elap_since_status_check;
        const uint32_t elap_since_status_check_interval = 5000;       // 5 seconds

        //-------------------------------------------------------------------------------------
        /// Throttling display intervals (ms) for status and ADC prints.
        elapsedMillis  elap_since_Status_display_ {0};
        const uint32_t elap_since_Status_display_interval_ {30 * 1000};
        //-------------------------------------------------------------------------------------
        elapsedMillis  elap_since_ADC_read_ {0};
        const uint32_t elap_since_ADC_read_interval_ {5000};                                        // milliseconds between ADC reads
        const uint32_t elap_since_ADC_start_interval_ {elap_since_ADC_read_interval_ - 1500};       // 1.5 seconds before ADC read to start one-shot
        elapsedMillis  elap_since_last_ADC_display_ {0};
        const uint32_t elap_since_last_ADC_display_interval_ {30 * 1000};       // 30 seconds between ADC value displays
        // ADC control
        /// True when a one-shot ADC conversion has been started and not yet collected.
        bool adc_OneShot_in_progress_ {false};
        //------------------------------------------------------------------------------------
        /// Cached charging state and debounce timers.
        bool           is_charging_ {false};
        bool           should_start_charging();
        elapsedMillis  elap_since_charging_completed_ {0};
        const uint32_t elap_since_charging_completed_interval_ { 2 * 60 * 1000};       // 2 minutes
        //------------------------------------------------------------------------------------
        // ADC values
        /// Latest ADC values from the device (units noted in names).
        float ADC_IAC_value_amps_ {0.0f};
        float ADC_IBAT_value_amps_ {0.0f};
        float ADC_VAC_value_volts_ {0.0f};
        float ADC_VBAT_value_volts_ {0.0f};
        float ADC_VSYS_value_volts_ {0.0f};
        float ADC_TS_value_celsius_ {0.0f};
        float ADC_TS_value_percent_of_REGN_ {0.0f};
        // Private methods
        /// Start an ADC conversion (continuous or one-shot depending on configuration).
        void start_ADC_Conversion();
        // // bool check_if_ADC_conversion_complete();
        /// Read and cache all ADC values from the device.
        void read_all_ADC_values();
        /// Return true if input power is good per status flags.
        bool input_power_good();
        // --------------------------------------------------------
        /// Background handlers; return true if something changed.
        bool spin_ADC();                 // return true if something changed
        bool spin_status_change();       // return true if something changed
    };

}       // namespace bq25820

#endif       // BQ25820__BQ25820_HPP_
