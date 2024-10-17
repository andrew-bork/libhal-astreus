#pragma once

#include "debug.hpp"

#include <libhal/i2c.hpp>
#include <libhal-util/i2c.hpp>
#include <span>
#include <array>
#include "icm20948/icm20948_reg.hpp"
#include "vec3.hpp"
#include <numbers>



static constexpr float pi = std::numbers::pi_v<float>;
static constexpr float deg_t_rad = pi / 180.0f;
static constexpr float rad_t_deg = 180.0f / pi;
static constexpr float bit15limit = 32768.0f;
// static constexpr float g = 9.81f;
static constexpr float g = 1.0f;

class icm20948 {
    public: 

        enum class gyro_scale : hal::byte {
            dps_250 =0b000, // 250 degrees per second
            dps_500 =0b010, // 500 degrees per second
            dps_1000=0b100, // 1000 degrees per second
            dps_2000=0b110 // 2000 degrees per second
        };

        enum class accel_scale : hal::byte {
            g_2=0b000, // 2g
            g_4=0b010, // 4g
            g_8=0b100, // 8g
            g_16=0b110 // 16g
        };

        /**
         * @brief Gyro Averaging rate. A larger factor will result in more accurate readings at a slower output rate.
         * 
         */
        enum class gyro_averaging : hal::byte {
            x1=0,
            x2=1,
            x4=2,
            x8=3,
            x16=4,
            x32=5,
            x64=6,
            x128=7,
        };
        
        
        /**
         * @brief Accelerometer Averaging rate. A larger factor will result in more accurate readings at a slower output rate.
         * 
         */
        enum class accl_averaging : hal::byte {
            x1=0,
            x8=1,
            x16=2,
            x32=3,
        };

        
        /**
         * @brief Magnetometer power modes.
         * 
         */
        enum class magnetometer_power_mode : hal::byte {
            POWER_DOWN=0b0000'0000,
            SINGLE=0b0000'0001,
            HZ_10=0b0000'0010,
            HZ_20=0b0000'0100,
            HZ_50=0b0000'0110,
            HZ_100=0b0000'1000,
            SELF_TEST=0b0001'0000,
        };

        /**
         * @brief Construct a new icm20948 on a given bus.
         * 
         * @param p_bus 
         */
        icm20948(hal::i2c& p_bus) : m_bus(p_bus) {
            change_register_bank(icm20948_reg::BANK0);
            set_gyro_full_scale(gyro_scale::dps_500);
            set_accel_full_scale(accel_scale::g_4);
        }

        /**
         * @brief Read the who am i register
         */
        inline hal::byte who_am_i() {
            return register_read(icm20948_reg::who_am_i);
        }

        /**
         * @brief Soft reset the imu.
         * 
         */
        inline void reset() {
            register_write(icm20948_reg::pwr_mgmt_1, 0b1000'0000);
        }
        
        /**
         * @brief Put the imu in a low power mode.
         * 
         */
        inline void sleep() {
            hal::byte pwr_mng_1_content = register_read(icm20948_reg::pwr_mgmt_1);
            pwr_mng_1_content &= 0b0100'0000;
            register_write(icm20948_reg::pwr_mgmt_1, pwr_mng_1_content);

        }

        /**
         * @brief Wake the imu from low power mode.
         * 
         */
        inline void wake_up() {
            hal::byte pwr_mng_1_content = register_read(icm20948_reg::pwr_mgmt_1);
            pwr_mng_1_content &= 0b1011'1111;
            register_write(icm20948_reg::pwr_mgmt_1, pwr_mng_1_content);
        }

        /**
         * @brief Enable the acclerometer.
         * 
         */
        inline void enable_accelerometer() {
            hal::byte pwr_mng_2_content = register_read(icm20948_reg::pwr_mgmt_2);
            pwr_mng_2_content &= 0b1100'0111;
            register_write(icm20948_reg::pwr_mgmt_2, pwr_mng_2_content);

        }

        /**
         * @brief Disable the acclerometer.
         * 
         */
        inline void disable_accelerometer() {
            hal::byte pwr_mng_2_content = register_read(icm20948_reg::pwr_mgmt_2);
            pwr_mng_2_content |= 0b0011'1000;
            register_write(icm20948_reg::pwr_mgmt_2, pwr_mng_2_content);

        }

        /**
         * @brief Enable the gyroscope.
         * 
         */
        inline void enable_gyroscope() {
            hal::byte pwr_mng_2_content = register_read(icm20948_reg::pwr_mgmt_2);
            pwr_mng_2_content &= 0b1111'1000;
            register_write(icm20948_reg::pwr_mgmt_2, pwr_mng_2_content);

        }

        /**
         * @brief Disable the gyroscope.
         * 
         */
        inline void disable_gyroscope() {
            hal::byte pwr_mng_2_content = register_read(icm20948_reg::pwr_mgmt_2);
            pwr_mng_2_content |= 0b0000'0111;
            register_write(icm20948_reg::pwr_mgmt_2, pwr_mng_2_content);

        }



        // inline void set_power_mode(icm20948_reg::ak09916_power_mode p_mag_power_mode) {

        // }

        /**
         * @brief Enable the accelerometer and gyroscope.
         * 
         */
        inline void enable_all() {
            register_write(icm20948_reg::pwr_mgmt_2, 0b0000'0000);
        }
        /**
         * @brief Disable the accelerometer and gyroscope.
         * 
         */
        inline void disable_all() {
            register_write(icm20948_reg::pwr_mgmt_2, 0b0011'1111);
        }

        /**
         * @brief Return the current acceleration readings in m/s^2
         */
        inline math::vec3 acceleration() {
            std::array<hal::byte, 6> raw_data;
            register_read(icm20948_reg::accel_xout_h, raw_data);
            math::vec3 out;
            out.x = static_cast<float>(combine_signed(raw_data[0], raw_data[1])) * m_accelerometer_sensitivity;
            out.y = static_cast<float>(combine_signed(raw_data[2], raw_data[3])) * m_accelerometer_sensitivity;
            out.z = static_cast<float>(combine_signed(raw_data[4], raw_data[5])) * m_accelerometer_sensitivity;
            out -= m_accel_offset;
            return out;
        }
        /**
         * @brief Return the current angular rate readings in rad/s.
         */
        inline math::vec3 angular_rate() {
            std::array<hal::byte, 6> raw_data;
            register_read(icm20948_reg::gyro_xout_h, raw_data);
            math::vec3 out;
            out.x = static_cast<float>(combine_signed(raw_data[0], raw_data[1])) * m_gyroscope_sensivity;
            out.y = static_cast<float>(combine_signed(raw_data[2], raw_data[3])) * m_gyroscope_sensivity;
            out.z = static_cast<float>(combine_signed(raw_data[4], raw_data[5])) * m_gyroscope_sensivity;
            out -= m_gyro_offset;
            return out;
        }

        inline void read(math::vec3& acceleration, math::vec3& angular_rate) {
            std::array<hal::byte, 12> raw_data;
            register_read(icm20948_reg::accel_xout_h, raw_data);
            acceleration.x = static_cast<float>(combine_signed(raw_data[0], raw_data[1])) * m_accelerometer_sensitivity;
            acceleration.y = static_cast<float>(combine_signed(raw_data[2], raw_data[3])) * m_accelerometer_sensitivity;
            acceleration.z = static_cast<float>(combine_signed(raw_data[4], raw_data[5])) * m_accelerometer_sensitivity;
            acceleration -= m_accel_offset;

            // angular_rate.x = static_cast<float>(combine_signed(raw_data[6], raw_data[7])) * m_gyroscope_sensivity;
            // angular_rate.y = static_cast<float>(combine_signed(raw_data[8], raw_data[9])) * m_gyroscope_sensivity;
            // angular_rate.z = static_cast<float>(combine_signed(raw_data[10], raw_data[11])) * m_gyroscope_sensivity;
            angular_rate.x = static_cast<float>(combine_signed(raw_data[6], raw_data[7])) * m_gyroscope_sensivity;
            angular_rate.y = static_cast<float>(combine_signed(raw_data[8], raw_data[9])) * m_gyroscope_sensivity;
            angular_rate.z = static_cast<float>(combine_signed(raw_data[10], raw_data[11])) * m_gyroscope_sensivity;
            angular_rate -= m_gyro_offset;
        }
        

        /**
         * @brief Return the current magnetometer readings in uT
         */
        inline math::vec3 magnetic_field() {
            std::array<hal::byte, 8> raw_data; // Must read the ST2 register at the end? idk.
            magnetometer_register_read(icm20948_reg::ak09916_hxl, raw_data);
            math::vec3 out;
            out.x = static_cast<float>(combine_signed(raw_data[1], raw_data[0])) * m_magnetometer_sensivity;
            out.y = static_cast<float>(combine_signed(raw_data[3], raw_data[2])) * m_magnetometer_sensivity;
            out.z = static_cast<float>(combine_signed(raw_data[5], raw_data[4])) * m_magnetometer_sensivity;
            return out;

        }

        /**
         * @brief Set the dlpf sample rate of the gyroscope.
         */
        inline void set_dlpf_gyro_sample_rate(float p_sample_rate) {
            float clock_freq = 1125.0f;
            unsigned int clock_div = static_cast<unsigned int>((clock_freq / p_sample_rate) - 1.0f); 
            if(255 < clock_div) {
                throw hal::argument_out_of_domain(this);
            }
            change_register_bank(icm20948_reg::BANK2);
            register_write(icm20948_reg::gyro_smplrt_div, clock_div);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Set the dlpf configuration of the gyroscope.
         */
        inline void set_gyro_dlpf_config(hal::byte p_dlpf_config) {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte gyro_config_1_data = register_read(icm20948_reg::gyro_config_1);
            if(8 <= p_dlpf_config) {
                throw hal::argument_out_of_domain(this);
            }
            gyro_config_1_data &= 0b1100'0111;
            gyro_config_1_data |= p_dlpf_config << 3;
            register_write(icm20948_reg::gyro_config_1, gyro_config_1_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Set the full scale of the gyroscope.
         */
        inline void set_gyro_full_scale(gyro_scale p_scale) {
            switch(p_scale) {
            case gyro_scale::dps_250:
                m_gyroscope_sensivity = (1 * 250.0f * deg_t_rad / bit15limit);
                break;
            case gyro_scale::dps_500:
                m_gyroscope_sensivity = (1 * 500.0f * deg_t_rad / bit15limit);
                break;
            case gyro_scale::dps_1000:
                m_gyroscope_sensivity = (1 * 1000.0f * deg_t_rad / bit15limit);
                break;
            case gyro_scale::dps_2000:
                m_gyroscope_sensivity =(1 * 2000.0f * deg_t_rad / bit15limit);
                break;
            }
            change_register_bank(icm20948_reg::BANK2);
            hal::byte gyro_config_1_data = register_read(icm20948_reg::gyro_config_1);
            gyro_config_1_data &= 0b1111'1001;
            gyro_config_1_data |= static_cast<hal::byte>(p_scale);
            register_write(icm20948_reg::gyro_config_1, gyro_config_1_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Enable the dlpf of the gyroscope.
         */
        inline void enable_gyro_dlpf() {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte gyro_config_1_data = register_read(icm20948_reg::gyro_config_1);
            gyro_config_1_data |= 0b0000'0001;
            register_write(icm20948_reg::gyro_config_1, gyro_config_1_data);
            change_register_bank(icm20948_reg::BANK0);
        }
        
        /**
         * @brief Disable the dlpf of the gyroscope.
         */
        inline void disable_gyro_dlpf() {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte gyro_config_1_data = register_read(icm20948_reg::gyro_config_1);
            gyro_config_1_data &= 0b1111'1110;
            register_write(icm20948_reg::gyro_config_1, gyro_config_1_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Configure the gyroscope averaging factor.
         * 
         * @param p_setting 
         */
        inline void set_gyro_averaging(gyro_averaging p_setting) {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte gyro_config_2_data = register_read(icm20948_reg::gyro_config_2);
            gyro_config_2_data &= 0b1111'1000;
            gyro_config_2_data |= static_cast<hal::byte>(p_setting);
            register_write(icm20948_reg::gyro_config_2, gyro_config_2_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        // inline void configure_gyro() {
            
        // }



        /**
         * @brief Set the dlpf sample rate of the accelerometer.
         */
        inline void set_dlpf_accel_sample_rate(float p_sample_rate) {
            float clock_freq = 1125.0f;
            unsigned int clock_div = static_cast<unsigned int>((clock_freq / p_sample_rate) - 1.0f); 
            if(0b1111'1111'1111 < clock_div) {
                throw hal::argument_out_of_domain(this);
            }
            change_register_bank(icm20948_reg::BANK2);
            register_write(icm20948_reg::accel_smplrt_div_1, clock_div >> 8);
            register_write(icm20948_reg::accel_smplrt_div_2, clock_div & 0b1111'1111);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Set the dlpf configuration of the accelerometer.
         */
        inline void set_accel_dlpf_config(hal::byte p_dlpf_config) {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte accel_config_data = register_read(icm20948_reg::accel_config);
            if(8 <= p_dlpf_config) {
                throw hal::argument_out_of_domain(this);
            }
            accel_config_data &= 0b1100'0111;
            accel_config_data |= p_dlpf_config << 3;
            register_write(icm20948_reg::accel_config, accel_config_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Set the full scale of the accelerometer.
         */
        inline void set_accel_full_scale(accel_scale p_scale) {
            switch(p_scale) {
            case accel_scale::g_2:
                m_accelerometer_sensitivity = 2.0f * g / bit15limit;
                break;
            case accel_scale::g_4:
                m_accelerometer_sensitivity = 4.0f * g / bit15limit;
                break;
            case accel_scale::g_8:
                m_accelerometer_sensitivity = 8.0f * g / bit15limit;
                break;
            case accel_scale::g_16:
                m_accelerometer_sensitivity = 16.0f * g / bit15limit;
                break;
            }
            change_register_bank(icm20948_reg::BANK2);
            hal::byte accel_config_data = register_read(icm20948_reg::accel_config);
            debug_log<128>("original accel config: %02x", accel_config_data);
            accel_config_data &= 0b1111'1001;
            accel_config_data |= static_cast<hal::byte>(p_scale);
            debug_log<128>("new accel config: %02x", accel_config_data);
            register_write(icm20948_reg::accel_config, accel_config_data);
            accel_config_data = register_read(icm20948_reg::accel_config);
            debug_log<128>("current accel config: %02x", accel_config_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Enable the dlpf of the accelerometer.
         */
        inline void enable_accel_dlpf() {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte accel_config_data = register_read(icm20948_reg::accel_config);
            accel_config_data |= 0b0000'0001;
            register_write(icm20948_reg::accel_config, accel_config_data);
            change_register_bank(icm20948_reg::BANK0);
        }
        
        /**
         * @brief Disable the dlpf of the accelerometer.
         */
        inline void disable_accel_dlpf() {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte accel_config_data = register_read(icm20948_reg::accel_config);
            accel_config_data &= 0b1111'1110;
            register_write(icm20948_reg::accel_config, accel_config_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Configure the accelerometer averaging factor.
         * 
         * @param p_setting 
         */
        inline void set_accel_averaging(accl_averaging p_setting) {
            change_register_bank(icm20948_reg::BANK2);
            hal::byte accel_config_2_data = register_read(icm20948_reg::accel_config_2);
            accel_config_2_data &= 0b1111'1100;
            accel_config_2_data |= static_cast<hal::byte>(p_setting);
            register_write(icm20948_reg::accel_config_2, accel_config_2_data);
            change_register_bank(icm20948_reg::BANK0);
        }

        /**
         * @brief Set the magnetometer power mode. This also configures the sample rate of the magnetometer.
         * 
         */
        inline void set_mag_power_mode(magnetometer_power_mode p_mode) {
            magnetometer_register_write(icm20948_reg::ak09916_cntl_2, static_cast<hal::byte>(p_mode));
        }

        /**
         * @brief Reset the magnetometer.
         */
        inline void reset_magnetometer() {
            magnetometer_register_write(icm20948_reg::ak09916_cntl_3, 1);
        }


        
        inline void calibrate_gyro(hal::steady_clock& p_clk, int p_iterations, hal::time_duration p_delay) {
            math::vec3 sum;
            for(int i = 0; i < p_iterations; i ++) {
                sum += angular_rate();
                hal::delay(p_clk, p_delay);
            }
            m_gyro_offset = sum / static_cast<float>(p_iterations);
        }
        
        inline void calibrate_accel(hal::steady_clock& p_clk, int p_iterations, hal::time_duration p_delay) {
            math::vec3 sum;
            for(int i = 0; i < p_iterations; i ++) {
                sum += acceleration();
                hal::delay(p_clk, p_delay);
            }
            m_accel_offset = (sum / static_cast<float>(p_iterations)) - math::vec3(0.0f, 0.0f, 1.0f);
        }
        
        inline void calibrate_accel_gyro(hal::steady_clock& p_clk, int p_iterations, hal::time_duration p_delay) {
            math::vec3 g_sum, a_sum;
            for(int i = 0; i < p_iterations; i ++) {
                g_sum += angular_rate();
                a_sum += acceleration();
                hal::delay(p_clk, p_delay);
            }
            m_gyro_offset = g_sum / static_cast<float>(p_iterations);
            m_accel_offset = (a_sum / static_cast<float>(p_iterations)) - math::vec3(0.0f, 0.0f, 1.0f);
        }

    private:
    public:
        hal::byte m_i2c_addr = icm20948_reg::icm20948_address;
        hal::byte m_mag_i2c_addr = icm20948_reg::ak09916_address;
        hal::i2c& m_bus;

        math::vec3 m_gyro_offset, m_accel_offset;

        float m_accelerometer_sensitivity = 1.0f;
        float m_gyroscope_sensivity = 1.0f;
        float m_magnetometer_sensivity = 4912.0f / bit15limit;
    public:
        inline std::int16_t combine_signed(hal::byte high, hal::byte low) {
            return (static_cast<std::int16_t>(high) << 8) | static_cast<std::int16_t>(low);
        }

        inline void change_register_bank(icm20948_reg::register_bank p_bank) {
            register_write(icm20948_reg::reg_bank_sel, p_bank);
        }

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

        inline void magnetometer_register_read(hal::byte p_reg_addr, std::span<hal::byte> p_output) {
            hal::write_then_read(m_bus, m_mag_i2c_addr, std::span(&p_reg_addr, 1), p_output);
        }
        inline void magnetometer_register_write(hal::byte p_reg_addr, hal::byte p_data) {
            std::array<hal::byte, 2> transferred = { p_reg_addr, p_data };
            hal::write(m_bus, m_mag_i2c_addr, transferred);
        }

};