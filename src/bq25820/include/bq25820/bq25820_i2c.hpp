#pragma once

/**
 * @file bq25820_i2c.hpp
 * @brief Low-level I2C access helper for the TI BQ25820 charger.
 *
 * Provides basic device open/close and typed register read/write utilities used by the
 * higher-level BQ25820 driver. Errors are logged with rclcpp and surfaced via return
 * values or exceptions where appropriate.
 */

#include "bq25820_Registers.hpp"
#include <cstdint>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace bq25820
{

    /**
     * @brief Thin wrapper around Linux I2C ioctl calls for BQ25820 register access.
     */
    class BQ25820_I2C
    {
      public:
        int  count_of_consecutive_failed_I2C_Reads  = 0;
        int  count_of_consecutive_failed_I2C_Writes = 0;
        bool is_device_opened() const { return fd_ >= 0; }

      private:
        /// I2C device path (e.g., "/dev/i2c-1").
        std::string i2c_device_name_;
        /// 7-bit I2C address of the device.
        uint8_t i2c_address_;
        /// File descriptor for the opened I2C device, or -1 when closed.
        int fd_;

        /**
         * @brief Read a sequence of bytes starting at register address.
         * @param reg Starting register address.
         * @param buf Destination buffer.
         * @param len Number of bytes to read (must be > 0).
         * @return true on success, false on failure (also logs an error).
         * @throws std::runtime_error if the device is not opened.
         */
        bool read_regs(uint8_t reg, uint8_t *buf, size_t len)
        {
            if (fd_ < 0) return false;
            if (len == 0) return false;
            unsigned char sbuf[2] = {reg, 0};

            i2c_msg msgs[2];
            msgs[0].addr  = static_cast<uint16_t>(i2c_address_);
            msgs[0].flags = {};
            msgs[0].len   = static_cast<uint16_t>(1);
            msgs[0].buf   = sbuf;

            msgs[1].addr  = static_cast<uint16_t>(i2c_address_);
            msgs[1].flags = I2C_M_RD;
            msgs[1].len   = static_cast<uint16_t>(len);
            msgs[1].buf   = buf;

            i2c_rdwr_ioctl_data wrapper;       // = {.msgs = msgs, .nmsgs = 2};
            wrapper.msgs  = msgs;
            wrapper.nmsgs = 2;

            int  rc                               = ioctl(fd_, I2C_RDWR, &wrapper);
            bool ok                               = (rc >= 0) && (msgs[0].len == 1) && (msgs[1].len == len);
            count_of_consecutive_failed_I2C_Reads = ok ? 0 : count_of_consecutive_failed_I2C_Reads + 1;
            if ((!ok))
                {
                    if ((count_of_consecutive_failed_I2C_Reads < 3))
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to read  I2C registers, ioctl rc=%d  errno=%d  %s", rc, errno, strerror(errno));
                        }
                    return false;
                }
            return true;
        }

        /**
         * @brief Write a sequence of bytes starting at register address.
         * @param reg Starting register address.
         * @param buf Source buffer.
         * @param len Number of data bytes to write (1..32).
         * @return true on success, false on failure (also logs an error).
         * @throws std::runtime_error if the device is not opened.
         */
        bool write_regs(uint8_t reg, uint8_t *buf, size_t len)
        {
            if (fd_ < 0) throw std::runtime_error("I2C device not opened");
            if (len == 0) return false;
            if (len > 32) return false;

            uint8_t data[36];
            data[0] = reg;
            for (size_t i = 0; i < len; i++) data[i + 1] = buf[i];

            i2c_msg msg;
            msg.addr  = static_cast<uint16_t>(i2c_address_);
            msg.flags = {};
            msg.len   = static_cast<uint16_t>(len + 1);
            msg.buf   = data;

            i2c_rdwr_ioctl_data wrapper;
            wrapper.msgs  = &msg;
            wrapper.nmsgs = 1;

            int  rc                                = ioctl(fd_, I2C_RDWR, &wrapper);
            bool ok                                = (rc >= 0) && (msg.len == len + 1);
            count_of_consecutive_failed_I2C_Writes = ok ? 0 : count_of_consecutive_failed_I2C_Writes + 1;
            if (!ok)
                {
                    if (count_of_consecutive_failed_I2C_Writes < 3)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("BQ25820"), "Failed to write I2C registers, ioctl rc=%d  errno=%d  %s", rc, errno, strerror(errno));
                        }
                    return false;
                }
            return true;
        }

      public:
        /// Default constructor; device closed, address unset.
        BQ25820_I2C() : fd_(-1) {}
        /// Construct with device path and I2C address.
        BQ25820_I2C(const std::string &i2c_device_name, uint8_t address) : i2c_device_name_(i2c_device_name), i2c_address_(address), fd_(-1) {}

        /// Destructor closes the device if open.
        ~BQ25820_I2C() { close_device(); }

        /// Set I2C address (no I/O performed).
        void set_i2c_address(uint8_t address) { i2c_address_ = address; }
        /**
         * @brief Set the I2C device path. Closes the device if currently open.
         */
        void set_i2c_device_name(const std::string &device_name)
        {
            close_device();
            i2c_device_name_ = device_name;
        }

        /**
         * @brief Open the I2C device node.
         * @return true on success, false if open() failed.
         */
        bool open_device()
        {
            if (fd_ >= 0) close_device();
            fd_ = open(i2c_device_name_.c_str(), O_RDWR);
            return (fd_ >= 0);
        }

        /**
         * @brief Close the I2C device if open.
         */
        void close_device()
        {
            if (fd_ >= 0) close(fd_);
            fd_ = -1;
        }

        /**
         * @brief Read an 8-bit register.
         * @param reg Register address.
         * @return The byte value read.
         */
        uint8_t read_reg8(uint8_t reg)
        {
            uint8_t val = 0;
            bool    ok  = read_regs(reg, &val, 1);
            if (ok) return val;
            return 0;       // Return 0 if read failed
        }

        /// Read an 8-bit register using typed enum.
        uint8_t read_reg8(BQ25820_Register reg) { return read_reg8(static_cast<uint8_t>(reg)); }

        /**
         * @brief Read a 16-bit little-endian register pair starting at address.
         * @param reg Starting address of the low byte.
         * @return 16-bit value (little-endian).
         */
        uint16_t read_reg16(uint8_t reg)
        {
            union       // BQ25820 16-bit register is little-endian
            {
                uint8_t  bytes[2];
                uint16_t word;
            } U;
            U.word  = 0;
            bool ok = read_regs(reg, U.bytes, 2);
            if (ok) return U.word;
            return 0;       // Return 0 if read failed
        }
        /// Read a 16-bit register using typed enum.
        uint16_t read_reg16(BQ25820_Register reg) { return read_reg16(static_cast<uint8_t>(reg)); }

        /// Write an 8-bit register.
        bool write_reg8(uint8_t reg, uint8_t val) { return write_regs(reg, &val, 1); }
        /// Write an 8-bit register using typed enum.
        bool write_reg8(BQ25820_Register reg, uint8_t val) { return write_reg8(static_cast<uint8_t>(reg), val); }

        /**
         * @brief Write a 16-bit little-endian register pair starting at address.
         * @param reg Starting address of the low byte.
         * @param val 16-bit value to write (little-endian on the wire).
         * @return true on success, false on failure.
         */
        bool write_reg16(uint8_t reg, uint16_t val)
        {
            union       // BQ25820 16-bit register is little-endian
            {
                uint8_t  bytes[2];
                uint16_t word;
            } U;
            U.word  = val;
            bool ok = write_regs(reg, U.bytes, 2);
            return ok;
        }
        /// Write a 16-bit register using typed enum.
        bool write_reg16(BQ25820_Register reg, uint16_t val) { return write_reg16(static_cast<uint8_t>(reg), val); }

    };       // class BQ25820_I2C

}       // namespace bq25820