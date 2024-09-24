#pragma once
// #include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include "resource_list.hpp"


extern resource_list hardware;
void application();

void bus_scan(hal::steady_clock& clock, hal::serial& console, hal::i2c& bus);