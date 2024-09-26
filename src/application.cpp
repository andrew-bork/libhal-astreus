#include "application.hpp"




#include "icm20948.hpp"
#include "neo-m9n.hpp"





void application()
{
  using namespace std::chrono_literals;

  // auto& led = *hardware.status_led.value();
  auto& clock = *hardware.clock.value();
  auto& console = *hardware.console.value();
  auto& i2c = *hardware.i2c;

  // icm20948 imu(i2c);
  // auto result = imu.acceleration();

  neo_m9n gps(console);
  gps.update();


  bus_scan(clock, console, i2c);

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
