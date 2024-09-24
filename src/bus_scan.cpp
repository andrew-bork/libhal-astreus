#include "application.hpp"

#include <array>
#include <libhal-util/i2c.hpp>




void bus_scan(hal::steady_clock& clock, hal::serial& console, hal::i2c& bus) {
    using namespace std::chrono_literals;

    for(int i = 0; i < 255; i ++) {
        if(i & 0xF == 0) {
            hal::print<256>(console, "0x%x- ", (i & 0xF0));
        }
        try {
            std::array<hal::byte, 0> out = {};
            hal::write(bus, i, out, hal::create_timeout(clock, 500us));
            hal::print<256>(console, " 0x%02x", i);
        }catch(const hal::no_such_device& p_error) {
            // continue;
            hal::print(console, "   --  ");
        }catch(const hal::timed_out& p_error) {
            // continue;
            hal::print(console, "   --  ");
        }
    }
}