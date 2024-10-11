// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include "resource_list.hpp"

#include "application.hpp"

// This is only global so that the terminate handler can use the hardware
// provided.

resource_list hardware;

[[noreturn]] void terminate_handler() noexcept
{
  bool valid = hardware.status_led && hardware.clock;
  auto& console = *hardware.console.value();
  hal::print(console, "CRASHED\n");

  if (not valid) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern, and wait for the debugger.
  // On GDB, use the `where` command to see if you have the `terminate_handler`
  // in your stack trace.

  // auto& console

  auto& led = *hardware.status_led.value();
  auto& clock = *hardware.clock.value();

  while (true) {
    using namespace std::chrono_literals;
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 100ms);
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 1000ms);
  }
}

int main()
{
  try {
    hardware = initialize_platform();
  } catch (...) {
    while (true) {
      // halt here and wait for a debugger to connect
      continue;
    }
  }

  hal::set_terminate(terminate_handler);

  try {
    application();
  } catch (std::bad_optional_access const& e) {
    if (hardware.console) {
      hal::print(*hardware.console.value(),
                 "A resource required by the application was not available!\n"
                 "Calling terminate!\n");
    }
  } 
  hal::print(*hardware.console.value(),
              "efwaljl;kdsajfkldasjfkl;dsjafkl;dsaj!\n"
              "Calling terminate!\n");
   // Allow any other exceptions to terminate the application

  // Terminate if the code reaches this point.
  std::terminate();
}