#include "application.hpp"




#include "icm20948/icm20948.hpp"
#include "neo-m9n.hpp"
#include "mpl3115/mpl3115.hpp"

#include <libhal-util/serial.hpp>



void application()
{
  using namespace std::chrono_literals;

  // auto& led = *hardware.status_led.value();
  auto& clock = *hardware.clock.value();
  auto& console = *hardware.console.value();
  auto& i2c = *hardware.i2c;

  bus_scan(clock, console, i2c);

  icm20948 imu(i2c);
  hal::print<512>(console, "ICM20948 WHO AM I: %02x\n", imu.who_am_i());
  imu.reset();
  imu.reset_magnetometer();
  imu.set_accel_full_scale(icm20948::accel_scale::g_8);
  imu.set_gyro_full_scale(icm20948::gyro_scale::dps_1000);
  imu.enable_accel_dlpf();
  // imu.set_accel_dlpf_config()
  imu.enable_all();
  imu.set_mag_power_mode(icm20948::magnetometer_power_mode::HZ_100);
  auto result = imu.acceleration();

  mpl3115 barometer(i2c);
  hal::print<512>(console, "MPL3115 WHO AM I: %02x\n", barometer.who_am_i());
  barometer.reset();
  barometer.set_mode(mpl3115::measure_mode::ALTIMETER);

  

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
