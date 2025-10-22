// gpiochip4  
#ifndef BQ25820_GPIO_HPP
#define BQ25820_GPIO_HPP


#include <string>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#include <gpiod.h>

#include <rclcpp/rclcpp.hpp>


namespace bq25820
{

    class BQ25820_GPIO
    {
      private:
        gpiod_chip *chip_ = nullptr;
        gpiod_line *line_10_ = nullptr;
        gpiod_line *line_11_ = nullptr;
        gpiod_line *line_12_ = nullptr;
        gpiod_line *line_13_ = nullptr;
        gpiod_line *line_16_ = nullptr;

        std::string gpio_chip_name_;

      public:
        BQ25820_GPIO() : gpio_chip_name_("gpiochip4") {}
        BQ25820_GPIO(const std::string &gpio_chip_name) : gpio_chip_name_(gpio_chip_name) {}

        ~BQ25820_GPIO() { close_device(); }

        int get_gpio_10() 
        {
            if (!line_10_) open_device();
            if (!line_10_) throw std::runtime_error("GPIO lines not initialized");
            int value = gpiod_line_get_value(line_10_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO INT line value");
                throw std::runtime_error("Failed to read GPIO INT line value");
            }
            return value;
        }

        int get_gpio_11() 
        {
            if (!line_11_) open_device();
            if (!line_11_) throw std::runtime_error("GPIO lines not initialized");
            int value = gpiod_line_get_value(line_11_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO STAT1 line value");
                throw std::runtime_error("Failed to read GPIO STAT1 line value");
            }
            return value;
        }
        int get_gpio_12() 
        {
            if (!line_12_) open_device();
            if (!line_12_) throw std::runtime_error("GPIO lines not initialized");
            int value = gpiod_line_get_value(line_12_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO STAT2 line value");
                throw std::runtime_error("Failed to read GPIO STAT2 line value");
            }
            return value;
        }
        int get_gpio_13() 
        {
            if (!line_13_) open_device();
            if (!line_13_) throw std::runtime_error("GPIO lines not initialized");
            int value = gpiod_line_get_value(line_13_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO PG line value");
                throw std::runtime_error("Failed to read GPIO PG line value");
            }
            return value;
        }
        int get_gpio_16() 
        {
            if (!line_16_) open_device();
            if (!line_16_) throw std::runtime_error("GPIO lines not initialized");
            int value = gpiod_line_get_value(line_16_);
            if (value < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read GPIO CE line value");
                throw std::runtime_error("Failed to read GPIO CE line value");
            }
            return value;
        }

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
            return true;
        }

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