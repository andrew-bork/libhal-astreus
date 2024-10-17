#include "application.hpp"
#include "math.hpp"
#include "debug.hpp"


#include "icm20948/icm20948.hpp"
#include "neo-m9n.hpp"
#include "mpl3115/mpl3115.hpp"

#include <libhal-util/serial.hpp>
#define r_t_d 57.2957795131 


#include "mission_control.hpp"

mission_control* debug;

void application()
{
  using namespace std::chrono_literals;

  auto& led = *hardware.status_led.value();
  auto& clock = *hardware.clock.value();
  auto& console = *hardware.console.value();
  auto& i2c = *hardware.i2c;

  mission_control mc(console);
  debug = &mc;

  // bus_scan(clock, console, i2c);
  // hal::print<512>(console, "ICM20948 tsest\n");

  icm20948 imu(i2c);
  // hal::print<512>(console, "ICM20948 WHO AM I: %02x\n", imu.who_am_i());
  imu.reset();
  hal::delay(clock, 1ms);
  // hal::print<512>(console, "A\n");

  // imu.reset_magnetometer();
  // hal::print<512>(console, "B\n");
  mc.log("Configuring Sensors");
  imu.set_accel_full_scale(icm20948::accel_scale::g_16);
  // imu.set_accel_full_scale(icm20948::accel_scale::g_16);
  // imu.set_accel_full_scale(icm20948::accel_scale::g_16);
  // imu.set_accel_full_scale(icm20948::accel_scale::g_16);
  // hal::print<512>(console, "C\n");

  imu.set_gyro_full_scale(icm20948::gyro_scale::dps_250);
  // imu.set_gyro_full_scale(icm20948::gyro_scale::dps_250);
  // imu.set_gyro_full_scale(icm20948::gyro_scale::dps_250);
  // imu.set_gyro_full_scale(icm20948::gyro_scale::dps_250);
  // hal::print<512>(console, "D\n");

  imu.enable_accel_dlpf();
  // hal::print<512>(console, "E\n");
  imu.enable_gyro_dlpf();
  imu.set_dlpf_gyro_sample_rate(10);
  // imu.set_gyro_dlpf_config(1);
  // imu.set_gyro_averaging(icm20948::gyro_averaging::x32);
  // // imu.set_accel_dlpf_config()
  imu.enable_all();
  // hal::print<512>(console, "F\n");
  imu.wake_up();

  imu.change_register_bank(icm20948_reg::BANK2);
  // imu.register_write(icm20948_reg::gyro_config_1, 0x1B);
  // imu.register_write(icm20948_reg::accel_config, 0x0D);
  mc.log<512>("Gyro Config 1: %02X", imu.register_read(icm20948_reg::gyro_config_1)); // 0b00011011 0x0B
  mc.log<512>("Gyro Config 2: %02X", imu.register_read(icm20948_reg::gyro_config_2));
  mc.log<512>("Accel Config 1: %02X", imu.register_read(icm20948_reg::accel_config)); // 0b00001101 0x0D
  mc.log<512>("Accel Config 2: %02X", imu.register_read(icm20948_reg::accel_config_2));
  imu.change_register_bank(icm20948_reg::BANK0);
  mc.log<512>("Power Mng  1: %02X", imu.register_read(icm20948_reg::pwr_mgmt_1));
  mc.log<512>("Power Mng 2: %02X", imu.register_read(icm20948_reg::pwr_mgmt_2));
  // hal::print(console, "Calibrating...");
  mc.log("Calibrating");
  imu.calibrate_accel_gyro(clock, 1000, 1ms);
  mc.log("Finished Calibration");

  mc.log<512>("huh: %f %f %f", imu.m_gyro_offset.x, imu.m_gyro_offset.y, imu.m_gyro_offset.z);
  mc.log<512>("huh: %f %f %f", imu.m_accel_offset.x, imu.m_accel_offset.y, imu.m_accel_offset.z);
  // hal::print(console, "Finished Calibration.\n");
  // hal::print<512>(console, "K: %02x\n", imu.register_read(icm20948_reg::pwr_mgmt_1)); // 0b0100'0001
  // hal::print<512>(console, "C: %02x\n", imu.register_read(icm20948_reg::pwr_mgmt_2));
  // hal::print<512>(console, "d: %02x\n", imu.register_read(icm20948_reg::accel_config));
  // hal::print<512>(console, "L: %02x\n", imu.register_read(icm20948_reg::accel_config_2));
  // hal::print<512>(console, "a: %02x\n", imu.register_read(icm20948_reg::gyro_config_1));
  // hal::print<512>(console, "f: %02x\n", imu.register_read(icm20948_reg::gyro_config_2));

  // while(true);

  // imu.set_mag_power_mode(icm20948::magnetometer_power_mode::HZ_100);
  // hal::print<512>(console, "0x%02x\n", imu.register_read(icm20948_reg::accel_xout_h));

  // auto result = imu.acceleration();

  // mpl3115 barometer(i2c);
  // hal::print<512>(console, "MPL3115 WHO AM I: %02x\n", barometer.who_am_i());
  // barometer.reset();
  // barometer.set_mode(mpl3115::measure_mode::ALTIMETER);

  math::quarternion orientation(1.0f);


  std::uint64_t i = 0;
  float dt = 0.1f;
  float data_frame_dt = 0.1f;
  bool k = false;
  std::uint64_t dt_ticks = static_cast<std::uint64_t>(dt * clock.frequency());
  std::uint64_t then = clock.uptime();
  std::uint64_t data_frame_ticks = static_cast<std::uint64_t>(data_frame_dt / dt);
  math::vec3 position, velocity;
  while(true) {
    // std::uint64_t now = clock.uptime();
    // float dt = (now - then) / clock.frequency();

    // Update estimation.
    // auto rates = imu.angular_rate();
    math::vec3 body_acceleration, body_rates;
    imu.read(body_acceleration, body_rates);
    // body_rates = math::vec3(0*deg_t_rad, 10*deg_t_rad, 10*deg_t_rad);

    math::vec3 br = 0.5 * dt * body_rates;
    math::quarternion rate_quart;
    rate_quart.w = -br.x*orientation.x - br.y*orientation.y - br.z*orientation.z;
    rate_quart.x = br.x*orientation.w + br.z*orientation.y - br.y*orientation.z;
    rate_quart.y = br.y*orientation.w - br.z*orientation.x - br.x*orientation.z;
    rate_quart.z = br.z*orientation.w + br.y*orientation.x - br.x*orientation.y;


    math::vec3 acceleration = body_acceleration;
    acceleration.z -= 1.0f;
    position += (velocity + 0.5 * acceleration * dt) * dt;
    velocity += acceleration * dt;

    // math::quarternion rate_quart(1.0f, body_rates.x * 0.5 * dt, body_rates.y * 0.5 * dt, body_rates.z * 0.5 * dt);
    // orientation = rate_quart * orientation;
    orientation = orientation + rate_quart;
    orientation.norm();


    if(data_frame_ticks <= i) {
      i = 0;
      // auto euler = math::quarternion::to_euler_ZYX(orientation);
      // std::array<hal::byte, 12> a;
      // imu.register_read(icm20948_reg::gyro_xout_h, a);
      // hal::print<512>(console, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", a[0], a[1], a[2], a[3], a[4], a[5]);
      // hal::print<512>(console, "Roll: %4.1f°, Pitch: %4.1f°, Yaw: %.41f°\n", rates.x*r_t_d, rates.y*r_t_d, rates.z*r_t_d);

      // hal::write(console);
      // hal::print<512>(console, "Roll: %4.1f°, Pitch: %4.1f°, Yaw: %.41f°\n", euler.x*r_t_d, euler.y*r_t_d, euler.z*r_t_d);

      mission_control_data_frame data;
      data.time = clock.frequency() * clock.uptime();
      data.orientation = orientation;
      data.body_angular_rates = body_rates;
      data.body_acceleration = body_acceleration;
      data.acceleration = acceleration;
      data.velocity = velocity;
      data.position = position;

      mc.send_data_frame(data);
      // mc.log("Data frame sent");

      // start_frame(console);
      // send(console, orientation.x);
      // send(console, orientation.y);
      // send(console, orientation.z);
      // send(console, orientation.w);

      // send(console, rates.x);
      // send(console, rates.y);
      // send(console, rates.z);

      // end_frame(console);
      // console.flush();
      
      led.level(k);
      k = !k;
    }
    i++;


    std::uint64_t now = clock.uptime();
    do { now = clock.uptime(); }while((now - then) < dt_ticks); // Spinlock until dt seconds have passed;
  }

  // neo_m9n gps(console);
  // gps.update();

  // hal::print(console, "I2C\n");
  // hal::print(console, "Will reset after ~10 seconds\n");

  // for (int i = 0; i < 10; i++) {
  //   // Print message
  //   hal::print(console, "Hello, World\n");

  //   // Toggle LED
  //   led.level(true);
  //   hal::delay(clock, 500ms);

  //   led.level(false);
  //   hal::delay(clock, 500ms);
  // }

  // hal::print(console, "Resetting!\n");
  // hal::delay(clock, 100ms);

  // hardware.reset();
}
