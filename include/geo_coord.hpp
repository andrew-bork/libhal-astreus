#pragma once

#include "vec3.hpp"
#include <numbers>
#include <cmath>




struct geo_coord {
    // Latitude from Equator, Longitude from Prime Meridian, Altitude above sea Level
    float lat, lng, alt;
    

    /**
     * @brief Convert a Cartesion Coordinate into an estimated Geocoordinate.
     * 
     * @param v Point to convert
     * @return GeoCoordinate
     */
    static inline constexpr geo_coord cartesian_to_lat_lng_inaccurate(math::vec3 v) {
        // https://en.wikipedia.org/wiki/Spherical_coordinate_system
        constexpr float R = 6378137.0;
        float r = std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
        float cos_lat = v.z / r;
        float sin_lat = std::sqrt(1 - cos_lat * cos_lat);
        geo_coord out;
        out.alt = r - R;
        out.lat = std::acos(cos_lat);
        out.lng = std::acos(v.x / (r * sin_lat));
        return out;
    }

    /**
     * @brief Convert Geocoordinate to a Cartesian coordinate.
     * 
     * @param x Coordinate to Convert
     * @return Cartesion Coordinate
     */
    static inline constexpr math::vec3 lat_lng_to_cartesian(geo_coord x) {
        //https://stackoverflow.com/questions/1185408/converting-from-longitude-latitude-to-cartesian-coordinates

        constexpr float a = 6378137.0, // radius a of earth in meters cfr WGS84
                        b = 6356752.3; // radius b of earth in meters cfr WGS84

        float e2=1-(b*b/a*a);
        float latr = x.lat/90*0.5*std::numbers::pi_v<float>; // latitude in radians
        float lonr = x.lng/180*std::numbers::pi_v<float>;    // longituede in radians
        float sin_lat = std::sin(latr);
        float Nphi=a/std::sqrt(1-e2*sin_lat*sin_lat);
        
        math::vec3 out;
        out.x=(Nphi+x.alt)*cos(latr)*cos(lonr);
        out.y=(Nphi+x.alt)*cos(latr)*sin(lonr);
        out.z=(b*b/(a*a)*Nphi+x.alt)*sin(latr);
        return out;
    }


    // static inline constexpr float ground_dist_between(geo_coord a, geo_coord b) {

    // }
};