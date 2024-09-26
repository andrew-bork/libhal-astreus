#include "application.hpp"

#include <array>
#include <libhal-util/i2c.hpp>




void bus_scan(hal::steady_clock& clock, hal::serial& console, hal::i2c& bus) {
    using namespace std::chrono_literals;

    for(int i = 0; i < 255; i ++) {
        if((i & 0xF) == 0) {
            hal::print<256>(console, "0x%x- ", (i & 0xF0));
        }

        if(hal::probe(bus, i)) {
            hal::print<256>(console, " 0x%02x", i);
        }else{
            hal::print(console, "   --  ");
        }
    }
}