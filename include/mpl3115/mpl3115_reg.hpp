#pragma once 



#include <libhal-util/bit.hpp>
#include <libhal/units.hpp>



namespace mpl3115_reg {
static constexpr hal::byte DEFAULT_I2C_ADDR = 0xC0;

static constexpr hal::byte STATUS = 0x00;
static constexpr hal::byte OUT_P_MSB = 0x01;
static constexpr hal::byte OUT_T_MSB = 0x04;
static constexpr hal::byte DR_STATUS = 0x06;
static constexpr hal::byte OUT_P_DELTA_MSB = 0x07;
static constexpr hal::byte OUT_T_DELTA_MSB = 0x0A;
static constexpr hal::byte WHO_AM_I = 0x0C;
static constexpr hal::byte F_STATUS = 0x0D;
static constexpr hal::byte F_DATA = 0x0E;
static constexpr hal::byte F_SETUP = 0x0F;
static constexpr hal::byte TIME_DLY = 0x10;
static constexpr hal::byte SYSMOD = 0x11;
static constexpr hal::byte INT_SOURCE = 0x12;
static constexpr hal::byte PT_DATA_CFG = 0x13;
static constexpr hal::byte BAR_IN_MSB = 0x14;
static constexpr hal::byte BAR_IN_LSB = 0x15;
static constexpr hal::byte P_TGT_MSB = 0x16;
static constexpr hal::byte P_WND_MSB = 0x19;
static constexpr hal::byte T_WND = 0x1B;
static constexpr hal::byte P_MIN_MSB = 0x1C;
static constexpr hal::byte T_MIN_MSB = 0x1F;
static constexpr hal::byte P_MAX_MSB = 0x21;
static constexpr hal::byte T_MAX_MSB = 0x24;
static constexpr hal::byte CTRL_REG1 = 0x26;
static constexpr hal::byte CTRL_REG2 = 0x27;
static constexpr hal::byte CTRL_REG3 = 0x28;
static constexpr hal::byte CTRL_REG4 = 0x29;
static constexpr hal::byte CTRL_REG5 = 0x2A;
static constexpr hal::byte OFF_P = 0x2B;
static constexpr hal::byte OFF_T = 0x2C;
static constexpr hal::byte OFF_H = 0x2D;



};