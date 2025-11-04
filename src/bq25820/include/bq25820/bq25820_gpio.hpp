// gpiochip4  
#ifndef BQ25820_GPIO_HPP
#define BQ25820_GPIO_HPP

/**
 * @file bq25820_gpio.hpp
 * @brief GPIO access helper for the TI BQ25820 charger signals.
 *
 * Wraps libgpiod access to specific GPIO lines used by the BQ25820 (INT, STAT1,
 * STAT2, PG, CE). Provides simple getters that read line values and report
 * errors via rclcpp logging and exceptions.
 */


#include <string>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <cerrno>
#include <cstring>

#include <gpiod.h>

#include <rclcpp/rclcpp.hpp>


namespace bq25820
{

    /**
     * @brief Manages access to BQ25820-related GPIO lines using libgpiod.
     *
     * The class lazily opens the configured GPIO chip and fetches lines 10, 11, 12,
     * 13, and 16, which correspond to INT, STAT1, STAT2, PG, and CE respectively.
     * Getters throw std::runtime_error on failure and log details via rclcpp.
     */
    class BQ25820_GPIO
    {
      private:
        /// Opened GPIO chip handle (nullptr when closed).
        gpiod_chip *chip_ = nullptr;
        /// Line 10: INT.
        gpiod_line *line_10_ = nullptr;
        /// Line 11: STAT1.
        gpiod_line *line_11_ = nullptr;
        /// Line 12: STAT2.
        gpiod_line *line_12_ = nullptr;
        /// Line 13: PG (Power Good).
        gpiod_line *line_13_ = nullptr;
        /// Line 16: CE (Charge Enable).
        gpiod_line *line_16_ = nullptr;

        /// Name of the GPIO chip to open (e.g., "gpiochip4").
        std::string gpio_chip_name_;

      public:
        /**
         * @brief Construct with a default chip name ("gpiochip4").
         */
        BQ25820_GPIO() : gpio_chip_name_("gpiochip4") {}
        /**
         * @brief Construct with a specific GPIO chip name.
         * @param gpio_chip_name The chip name to open (e.g., "gpiochip0").
         */
        BQ25820_GPIO(const std::string &gpio_chip_name) : gpio_chip_name_(gpio_chip_name) {}

        /// Destructor closes the device and releases lines.
        ~BQ25820_GPIO() { close_device(); }

        /**
         * @brief Read GPIO line 10 (INT).
         * @return 0 or 1 logic level.
         * @throws std::runtime_error on initialization or read failures.
         */
        int get_gpio_10() 
        {
            if (!line_10_) open_device();
            if (!line_10_) return -2;
            gpiod_line_request_input(line_10_, "BQ25820");
            int value = gpiod_line_get_value(line_10_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO INT line value  rc=%d   errno=%d  %s", value, errno, strerror(errno));
                //throw std::runtime_error("Failed to read GPIO INT line value");
            }
            return value;
        }

        /**
         * @brief Read GPIO line 11 (STAT1).
         * @return 0 or 1 logic level.
         * @throws std::runtime_error on initialization or read failures.
         */
        int get_gpio_11() 
        {
            if (!line_11_) open_device();
            if (!line_11_) return -2;
            gpiod_line_request_input(line_11_, "BQ25820");
            int value = gpiod_line_get_value(line_11_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO STAT1 line value  rc=%d   errno=%d  %s", value, errno, strerror(errno));
                //throw std::runtime_error("Failed to read GPIO STAT1 line value");
            }
            return value;
        }
        /**
         * @brief Read GPIO line 12 (STAT2).
         * @return 0 or 1 logic level.
         * @throws std::runtime_error on initialization or read failures.
         */
        int get_gpio_12() 
        {
            if (!line_12_) open_device();
            if (!line_12_) return -2;
            gpiod_line_request_input(line_12_, "BQ25820");
            int value = gpiod_line_get_value(line_12_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO STAT2 line value  rc=%d   errno=%d  %s", value, errno, strerror(errno));
                //throw std::runtime_error("Failed to read GPIO STAT2 line value");
            }
            return value;
        }
        /**
         * @brief Read GPIO line 13 (PG - Power Good).
         * @return 0 or 1 logic level.
         * @throws std::runtime_error on initialization or read failures.
         */
        int get_gpio_13() 
        {
            if (!line_13_) open_device();
            if (!line_13_) return -2;
            gpiod_line_request_input(line_13_, "BQ25820");
            int value = gpiod_line_get_value(line_13_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO PG line value  rc=%d   errno=%d  %s", value, errno, strerror(errno));
                //throw std::runtime_error("Failed to read GPIO PG line value");
            }
            return value;
        }
        /**
         * @brief Read GPIO line 16 (CE - Charge Enable).
         * @return 0 or 1 logic level.
         * @throws std::runtime_error on initialization or read failures.
         */
        int get_gpio_16() 
        {
            if (!line_16_) open_device();
            if (!line_16_) return -2;
            gpiod_line_request_input(line_16_, "BQ25820");
            int value = gpiod_line_get_value(line_16_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO CE line value  rc=%d   errno=%d  %s", value, errno, strerror(errno));
                //throw std::runtime_error("Failed to read GPIO CE line value");
            }
            return value;
        }

        /**
         * @brief Open the GPIO chip and fetch all required lines.
         * @return true on success, false if the chip or any line failed to open.
         */
        bool open_device()
        {
            chip_     = gpiod_chip_open_by_name(gpio_chip_name_.c_str());
            if (!chip_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to open GPIO chip: %s", gpio_chip_name_.c_str());
                return false;
            }

            line_10_ = gpiod_chip_get_line(chip_, 10);
            line_11_ = gpiod_chip_get_line(chip_, 11);
            line_12_ = gpiod_chip_get_line(chip_, 12);
            line_13_ = gpiod_chip_get_line(chip_, 13);
            line_16_ = gpiod_chip_get_line(chip_, 16);

            if (!line_10_ || !line_11_ || !line_12_ || !line_13_ || !line_16_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to get one or more GPIO lines");
                close_device();
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("BQ25820"), "Successfully opened GPIO lines");
            return true;
        }

        /**
         * @brief Release all GPIO lines and close the chip handle.
         */
        void close_device()
        {
            if (line_10_) gpiod_line_release(line_10_);
            if (line_11_) gpiod_line_release(line_11_);
            if (line_12_) gpiod_line_release(line_12_);
            if (line_13_) gpiod_line_release(line_13_);
            if (line_16_) gpiod_line_release(line_16_);
            if (chip_) gpiod_chip_close(chip_);
            line_10_ = nullptr;
            line_11_ = nullptr;
            line_12_ = nullptr;
            line_13_ = nullptr;
            line_16_ = nullptr;
            chip_    = nullptr;
        }
    };

} // namespace bq25820

#endif // BQ25820_GPIO_HPP