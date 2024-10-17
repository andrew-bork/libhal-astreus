#pragma once

#include "mission_control.hpp"






extern mission_control* debug; 

inline void debug_log(std::span<const char> p_data) {
    debug->log(p_data);
}
template<std::size_t N, class ...T>
void debug_log(const char * fmt, const T&... x) {
    debug->log<N>(fmt, x...);
}