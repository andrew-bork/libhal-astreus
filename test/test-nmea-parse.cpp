#include <array>
#include <span>
#include <string>
#include <cstdio>
#include <fstream>
#include <string_view>
#include <iostream>
#include "geo_coord.hpp"

struct nmea_parser {
    public:
    /**
     * @brief Parse NMEA Messages contained from a Serial Stream.
     * 
     * @param data Data from a serial stream.
     * @return True if a NMEA Message was fully parsed. False otherwise.
     */
    bool parse(std::span<const char> data) {
        bool data_updated = false;

        for(auto i = data.begin(); i != data.end(); i ++) {
            if(_buffer_pos == _internal_buffer.begin()) {
                if(*i == '$') {
                    *_buffer_pos = *i;
                    _buffer_pos++;
                }
            }else{
                *_buffer_pos = *i;
                _buffer_pos++;
                
                if(*(_buffer_pos-1) == '\n' && *(_buffer_pos-2) == '\r') {
                    data_updated = true;
                    process_buffered_message();
                }
            }
        }

        return data_updated;
    }

    /**
     * @brief Empty the internal buffer. 
     */
    inline void flush() {
        _buffer_pos = _internal_buffer.begin();
    }

    /**
     * @brief Returns true when the GPS has a position fix. False otherwise.
     */
    inline bool is_fixed() {
        return _last_gga_update.is_locked;
    }

    /**
     * @brief Returns the last GPS location recieved by the reciever.
     * @warning Valid only when the GPS is fixed.
     */
    inline geo_coord coord() {
        return _last_gga_update.coord;
    }

    private:
    /**
     * @brief Process the message stored in the internal buffer. 
     * Message is required to a complete NMEA Message.
     * 
     */
    inline void process_buffered_message() {
        (*(_buffer_pos-2)) = '\0';
        _buffer_pos = _internal_buffer.begin();
        std::string_view x = std::span(_internal_buffer).subspan(3, 3);
        if(x == "GGA") process_GGA();
    }

    /**
     * @brief Process the GGA message stored in the internal buffer.
     * Message has to be GGA.
     * 
     */
    inline void process_GGA() {
        char talker_id[3];

        int lat_degrees = 0;
        float lat_minutes = 0;
        char lat_direction = '?';
        int lng_degrees = 0;
        float lng_minutes;
        char lng_direction = '?';

        float unused;
        
        // https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
        // $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F
        std::sscanf(_internal_buffer.data(), "$%2sGGA,%f,%2d%f,%c,%3d%f,%c,%d,%d,%f,%f,%c,%f,%c,%f,%9s",
            talker_id,
            &_last_gga_update.utc_time,
            &lat_degrees,
            &lat_minutes,
            &lat_direction,
            &lng_degrees,
            &lng_minutes,
            &lng_direction,
            &_last_gga_update.fix_status,
            &_last_gga_update.satellites_used,
            &_last_gga_update.hdop,
            &_last_gga_update.altitude,
            &_last_gga_update.altitude_units,
            &_last_gga_update.height_of_geoid,
            &_last_gga_update.height_of_geoid_units,
            &unused,
            &_last_gga_update.dgps_station_id_checksum
        );

        _last_gga_update.coord.lat = 
            (lat_direction == 'S' ? -1 : 1) * 
            (static_cast<float>(lat_degrees) + lat_minutes/60.0f);
        _last_gga_update.coord.lng = 
            (lng_direction == 'W' ? -1 : 1) * 
            (static_cast<float>(lng_degrees) + lng_minutes/60.0f);
        _last_gga_update.coord.alt = _last_gga_update.altitude;

        _last_gga_update.is_locked = _last_gga_update.fix_status != -0;
    }


    /**
     * @brief Data recieved on a successful NMEA-GGA Message.
     * 
     */
    struct gps_parsed
    {
        bool is_locked = false;
        float utc_time;
        geo_coord coord;
        int fix_status;
        int satellites_used;
        float hdop;
        float altitude;
        char altitude_units;
        float height_of_geoid;
        char height_of_geoid_units;
        char time_since_last_dgps_update;
        char dgps_station_id_checksum[10];
    } _last_gga_update;


    std::array<char, 512> _internal_buffer;
    std::array<char, 512>::iterator _buffer_pos = _internal_buffer.begin();
};

int main() {
    nmea_parser parser;
    
    // parser.parse("");
    printf("Hello World!\n");
    std::ifstream t("../test-data/gps-test.txt");
    char buf[512];
    while(!t.eof()) {

        t.read(buf, 512);
        parser.parse(buf);
        auto loc = parser.coord();
        printf("%f,%f, %c\n", loc.lat, loc.lng, (parser.is_fixed() ? 'F' : 'N'));

    }
    // t >> buf;
    

    // const char * input = "ABC ABCDAB ABCDABCDABDE";
    // const char * pattern = "ABCDABD";
    // const char * input = "\r\r\noh no";
    // const char * pattern = "\r\n";
    // size_t i = test_find(input, pattern);

    // if(i == -1) {
    //     printf("\"%s\" not found in \"%s\"\n", pattern, input);
    // }else {
    //     printf("\"%s\" found in \"%s\" at i=%zu\n", pattern, input, i);
    // }
}
