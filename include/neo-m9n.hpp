#pragma once;

#include <libhal/serial.hpp>
#include <array>




class neo_m9n {
public:
    struct gps_parsed_t
    {
        bool is_locked = false;
        float time;
        float latitude;
        char latitude_direction;
        float longitude;
        char longitude_direction;
        int fix_status;
        int satellites_used;
        float hdop;
        float altitude;
        char altitude_units;
        float height_of_geoid;
        char height_of_geoid_units;
        char time_since_last_dgps_update;
        char dgps_station_id_checksum[10];
    };

    neo_m9n(hal::serial& p_serial);

    bool update();


private:
    hal::serial& m_serial;
    std::array<char, 512> m_msg_buffer;
    struct {
        bool is_fixed = false;
    } m_last_data;

//   hal::result<gps_parsed_t> read_raw_gps();
//   hal::result<gps_parsed_t> calculate_lon_lat(gps_parsed_t const& p_gps_data);
//   hal::result<gps_parsed_t> read();

// private:
//   neo_m9n(hal::serial& p_serial);
//   hal::serial* m_serial;
//   std::array<hal::byte, 512> m_gps_buffer;
//   gps_parsed_t m_gps_data;
// };
};