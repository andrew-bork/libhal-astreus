#pragma once


#include "mpl3115/mpl3115_reg.hpp"

#include <libhal/i2c.hpp>
#include <libhal-util/i2c.hpp>
#include <span>
#include <array>
#include "vec3.hpp"
#include <numbers>



/**
 * @brief https://www.nxp.com/docs/en/data-sheet/MPL3115A2.pdf 
 * 
 */
class mpl3115 {
    public:
        /**
         * @brief Construct a new mpl3115 on a given i2c bus.
         * 
         * @param p_bus 
         */
        mpl3115(hal::i2c& p_bus) : m_bus(p_bus) {
            set_oversampling(oversample::x2);
        }
        
        /**
         * @brief Barometer mode outputs pressure measurements.
         * Altimeter mode outputs height measurements
         * 
         */
        enum class measure_mode : hal::byte {
            BAROMETER=0b0000'0000, 
            ALTIMETER=0b1000'0000,
        };

        /**
         * @brief Trades off sensor data rate for sensor accurate.
         * A higher oversampling rate will result in a more accurate reading, at the cost of a slower sample period.
         * 
         */
        enum class oversample : hal::byte {
            x1 = 0b00'0000,    // 6ms
            x2 = 0b00'1000,    // 10ms
            x4 = 0b01'0000,    // 18ms
            x8 = 0b01'1000,    // 34ms
            x16 = 0b10'0000,   // 66ms
            x32 = 0b10'1000,   // 130ms
            x64 = 0b11'0000,   // 258ms
            x128 = 0b11'1000,  // 512ms
        };

        /**
         * @brief untested
         * 
         */
        struct status {
            hal::byte pressure_temperature_overwritten : 1; // The last Pressure/Height or Temperature reading has been overwritten.
            hal::byte pressure_overwritten : 1; // The last Pressure/Height reading has been overwritten.
            hal::byte temperature_overwritten : 1; // The last Temperature reading has been overwritten.
            hal::byte pressure_temperature_data_ready : 1; // A Pressure/Height or Temperature reading is ready to be read.
            hal::byte pressure_data_ready : 1; // A Pressure/Height reading is ready to be read.
            hal::byte temperature_data_ready : 1; // A Temperature reading is ready to be read.
        };

        /**
         * @brief Get the data ready status of the sensor
         * 
         * @return status 
         */
        inline status get_data_ready_status() {
            hal::byte status_data =register_read(mpl3115_reg::DR_STATUS);
            status out;
            out.pressure_temperature_overwritten = status_data & 0b1000'0000;
            out.pressure_overwritten = status_data & 0b0100'0000;
            out.temperature_overwritten = status_data & 0b0010'0000;
            out.pressure_temperature_data_ready = status_data & 0b0000'1000;
            out.pressure_data_ready = status_data & 0b0000'0100;
            out.temperature_data_ready = status_data & 0b0000'0010;
            return out;
        }


        /**
         * @brief Get the height reading from the sensor.
         * @warning Must be in Altimeter mode.
         * 
         * @return Height in meters above reference height.
         */
        inline float height() { 
            std::array<hal::byte, 3> raw_data;
            register_read(mpl3115_reg::OUT_P_MSB, raw_data);
            return static_cast<float>(combine_signed(raw_data[0], raw_data[1], raw_data[2] >> 4)) / 8.0f;
         }


        /**
         * @brief Get the pressure reading from the sensor.
         * @warning Must be in Barometer mode.
         * 
         * @return Pressure in Pascals.
         */
        inline float pressure() {
            std::array<hal::byte, 3> raw_data;
            register_read(mpl3115_reg::OUT_P_MSB, raw_data);
            return static_cast<float>(combine_unsigned(raw_data[0], raw_data[1], raw_data[2] >> 4));
        }

        /**
         * @brief Get the temperature reading from the sensor.
         * 
         * @return Temperature in Celcius.
         */
        inline float temperature() {
            std::array<hal::byte, 2> raw_data;
            register_read(mpl3115_reg::OUT_T_MSB, raw_data);
            return static_cast<float>(combine_signed(raw_data[0], raw_data[1] >> 4));
        }

        inline void read(float& p_pressure, float& p_temperature) {
            std::array<hal::byte, 5> raw_data;
            register_read(mpl3115_reg::OUT_P_MSB, raw_data);
            p_pressure = static_cast<float>(combine_unsigned(raw_data[0], raw_data[1], raw_data[2] >> 4));
            p_temperature = static_cast<float>(combine_signed(raw_data[3], raw_data[4] >> 4));
        }


        /**
         * @brief Get the difference in height from the current and last reading from the sensor.
         * @warning Must be in Altimeter mode.
         * 
         * @return Delta Height in meters.
         */
        inline float delta_height() { 
            std::array<hal::byte, 3> raw_data;
            register_read(mpl3115_reg::OUT_P_DELTA_MSB, raw_data);
            return static_cast<float>(combine_signed(raw_data[0], raw_data[1], raw_data[2] >> 4)) / 8.0f;
        }

        /**
         * @brief Get the difference in pressures from the current and last reading from the sensor.
         * @warning Must be in Barometer mode.
         * 
         * @return Delta Pressure in pascals.
         */
        inline float delta_pressure() {
            std::array<hal::byte, 3> raw_data;
            register_read(mpl3115_reg::OUT_P_DELTA_MSB, raw_data);
            return static_cast<float>(combine_unsigned(raw_data[0], raw_data[1], raw_data[2] >> 4));
        }

        /**
         * @brief Get the difference in temperature from the current and last reading from the sensor.
         * 
         * @return Delta Temperature in Celcius.
         */
        inline float delta_temperature() {
            std::array<hal::byte, 2> raw_data;
            register_read(mpl3115_reg::OUT_T_DELTA_MSB, raw_data);
            return static_cast<float>(combine_signed(raw_data[0], raw_data[1] >> 4));
        }


        inline void read_delta_with_pressure(float& p_delta_pressure, float& p_delta_temperature) {
            std::array<hal::byte, 5> raw_data;
            register_read(mpl3115_reg::OUT_P_DELTA_MSB, raw_data);
            p_delta_pressure = static_cast<float>(combine_unsigned(raw_data[0], raw_data[1], raw_data[2] >> 4));
            p_delta_temperature = static_cast<float>(combine_signed(raw_data[3], raw_data[4] >> 4));
        }
        inline void read_delta_with_height(float& p_delta_height, float& p_delta_temperature) {
            std::array<hal::byte, 5> raw_data;
            register_read(mpl3115_reg::OUT_P_DELTA_MSB, raw_data);
            p_delta_height = static_cast<float>(combine_signed(raw_data[0], raw_data[1], raw_data[2] >> 4)) / 8.0f;
            p_delta_temperature = static_cast<float>(combine_signed(raw_data[3], raw_data[4] >> 4));
        }

        
        /**
         * @brief Return the current sensor sample rate based on oversampling setting.
         * 
         * @return Sample rate in seconds
         */
        inline float sample_period() {
            return m_sample_period;
        }

        /**
         * @brief Return the vertical speed.
         * @warning Must be in Altimeter mode.
         * 
         * @return V/S in m/s
         */
        inline float vertical_speed() {
            return delta_height() / m_sample_period;
        }

        /**
         * @brief Query the who am i register.
         */
        inline hal::byte who_am_i() {
            return register_read(mpl3115_reg::WHO_AM_I);
        }

        /**
         * @brief Put the sensor into standby mode.
         * 
         */
        inline void sleep() {
            hal::byte value = register_read(mpl3115_reg::CTRL_REG1);
            value &= 0b1111'1110;
            register_write(mpl3115_reg::CTRL_REG1, value);
        }
        /**
         * @brief Put the sensor into active mode.
         * 
         */
        inline void wake_up() {
            hal::byte value = register_read(mpl3115_reg::CTRL_REG1);
            value |= 1;
            register_write(mpl3115_reg::CTRL_REG1, value);
        }

        /**
         * @brief Set the reference pressure for height calculations
         * 
         * @param p_pressure Reference pressure in Pascals.
         */
        inline void set_reference_pressure(float p_pressure) {
            std::uint16_t data = static_cast<std::uint16_t>(p_pressure / 2.0f);
            register_write(mpl3115_reg::BAR_IN_MSB, data >> 8);
            register_write(mpl3115_reg::BAR_IN_LSB, data & 0xff);
        }

        /**
         * @brief Set the current measuring mode. 
         * 
         * @param p_mode ALTIMETER for height measurements, BAROMETER for pressure measurements.
         */
        inline void set_mode(measure_mode p_mode) {
            hal::byte value = register_read(mpl3115_reg::CTRL_REG1);
            value &= 0b0111'1111;
            value |= static_cast<hal::byte>(p_mode);
            register_write(mpl3115_reg::CTRL_REG1, value);
        }

        /**
         * @brief Soft reset the sensor.
         * 
         */
        inline void reset() {
            hal::byte value = register_read(mpl3115_reg::CTRL_REG1);
            value |= 0b0000'0100;
            register_write(mpl3115_reg::CTRL_REG1, value);
        }

        /**
         * @brief Set the oversampling setting of the sensor
         * 
         * @param p_oversample Oversampling rate.
         */
        inline void set_oversampling(oversample p_oversample) {
            switch(p_oversample) {
            case oversample::x1:
                m_sample_period = 0.006f;
                break;
            case oversample::x2:
                m_sample_period = 0.010f;
                break;
            case oversample::x4:
                m_sample_period = 0.018f;
                break;
            case oversample::x8:
                m_sample_period = 0.034f;
                break;
            case oversample::x16:
                m_sample_period = 0.066f;
                break;
            case oversample::x32:
                m_sample_period = 0.130f;
                break;
            case oversample::x64:
                m_sample_period = 0.258f;
                break;
            case oversample::x128:
                m_sample_period = 0.512f;
                break;
            }
            hal::byte value = register_read(mpl3115_reg::CTRL_REG1);
            value &= 0b1100'0111;
            value |= static_cast<hal::byte>(p_oversample);
            register_write(mpl3115_reg::CTRL_REG1, value);
        }

    private:


        hal::byte m_i2c_addr = mpl3115_reg::DEFAULT_I2C_ADDR;
        hal::i2c& m_bus;
        float m_sample_period = 0.006f;



        // float m_accelerometer_sensitivity = 1.0f;
        // float m_gyroscope_sensivity = 1.0f;
        // float m_magnetometer_sensivity = 4912.0f / bit15limit;

        /**
         * @brief Combine into unsigned 20 bit number
         * 
         * AAAAAAAABBBBBBBBCCCC
         * 
         * @param high 
         * @param mid 
         * @param low  
         * @return std::uint32_t 
         */
        inline std::uint32_t combine_unsigned(hal::byte high, hal::byte mid, hal::byte low) {
            return (static_cast<std::uint32_t>(high) << 12) | (static_cast<std::uint32_t>(mid) << 4) | static_cast<std::uint32_t>(low);
        }

        /**
         * @brief Combine into signed 20 bit number (twos complement)
         * 
         * AAAAAAAABBBBBBBBCCCC
         * 
         * @param high 
         * @param mid 
         * @param low  
         * @return std::uint32_t 
         */
        inline std::uint32_t combine_signed(hal::byte high, hal::byte mid, hal::byte low) {
            return (static_cast<std::uint32_t>(high << 24) >> 12) | (static_cast<std::uint32_t>(mid) << 4) | static_cast<std::uint32_t>(low);
        }
        
        
        /**
         * @brief Combine into signed 16 bit number (twos complement)
         * 
         * AAAAAAAABBBBBBBBCCCC
         * 
         * @param high 
         * @param mid 
         * @param low  
         * @return std::uint32_t 
         */
        inline std::int16_t combine_signed(hal::byte high, hal::byte low) {
            return (static_cast<std::int16_t>(high) << 8) | static_cast<std::int16_t>(low);
        }

        // inline void change_register_bank(icm20948_reg::register_bank p_bank) {
        //     register_write(icm20948_reg::reg_bank_sel, p_bank);
        // }

        // inline std::uint16_t combine_unsigned(hal::byte high, hal::byte low) {
        //     return (static_cast<std::uint16_t>(high) << 8) | static_cast<std::uint16_t>(low);
        // }

        inline void register_write(hal::byte p_reg_addr, hal::byte p_data) {
            std::array<hal::byte, 2> transferred = { p_reg_addr, p_data };
            hal::write(m_bus, m_i2c_addr, transferred);
        }

        inline hal::byte register_read(hal::byte p_reg_addr) {
            hal::byte output;
            hal::write_then_read(m_bus, m_i2c_addr, std::span(&p_reg_addr, 1), std::span(&output, 1));
            return output;
        }

        inline void register_read(hal::byte p_reg_addr, std::span<hal::byte> p_output) {
            hal::write_then_read(m_bus, m_i2c_addr, std::span(&p_reg_addr, 1), p_output);
        }
};