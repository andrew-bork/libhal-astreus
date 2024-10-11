#pragma once

#include <libhal/serial.hpp>
#include <array>
#include <math.hpp>
#include <vec3.hpp>


struct mission_control_data_frame {
    float time;
    math::vec3 position;
    math::vec3 velocity;
    math::vec3 acceleration;
    math::vec3 body_acceleration;

    math::quarternion orientation;
    math::vec3 body_angular_rates;
};

struct mission_control {
    hal::serial& m_console;
    mission_control(hal::serial& p_console) : m_console(p_console) {

    }

    enum frame_bytes : std::uint8_t {
        START_OF_FRAME=0x01, // Start of header
        END_OF_FRAME=0x04, // End of transmission
        ESCAPE=0x27,
    };

    inline void start_frame() {
        std::array<std::uint8_t, 1>  msg = { START_OF_FRAME };
        m_console.write(msg);
    }

    void send_byte(std::uint8_t b) {
        std::array<std::uint8_t, 1>  msg = { ESCAPE };
        switch(b) {
            case START_OF_FRAME:
            case END_OF_FRAME:
            case ESCAPE:
                m_console.write(msg);
                break;
            default:
                break;
        }
        msg[0] = b;
        m_console.write(msg);
    }

    inline void end_frame() {
        std::array<std::uint8_t, 1>  msg = { END_OF_FRAME };
        m_console.write(msg);
    }

    template <typename T>
    void send(T x) {
        union {
            T a;
            std::uint8_t bytes[sizeof(T)];
        } thing;
        thing.a = x;
        for(std::size_t i = 0; i <= sizeof(T); i ++) {
            send_byte(thing.bytes[i]);
        }
    }

    void send_v(math::vec3 x) {
        send(x.x);
        send(x.y);
        send(x.z);
    }
    void send_q(math::quarternion x) {
        send(x.x);
        send(x.y);
        send(x.z);
        send(x.w);
    }


    void send_data_frame(const mission_control_data_frame& p_frame) {
        start_frame();
        send('D');
        send(p_frame.time);
        send_v(p_frame.position);
        send_v(p_frame.velocity);
        send_v(p_frame.acceleration);
        send_v(p_frame.body_acceleration);
        send_q(p_frame.orientation);
        send_v(p_frame.body_angular_rates);
        end_frame();
    }

    void log(std::span<const char> p_data) {
        start_frame();
        send('L');
        for(const auto& byte : p_data) {
            send(byte);
        }
        end_frame();
    }
};

// struct mission_control_log_frame {
//     std::int64_t time;

// };