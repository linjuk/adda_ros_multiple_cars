#ifndef DESPOT_HELPER_FUNCTIONS_H
#define DESPOT_HELPER_FUNCTIONS_H

#include <cmath>
#include <ros/ros.h>

namespace geometry_msgs {

template <typename T>
double length(const T& p) {
    return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
}

template <typename T>
bool operator == (const T &p1, const T &p2) {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
}

template <typename T>
bool operator < (const T &p1, const T &p2) {
    return length(p1) < length(p2);
}

template <typename T>
bool operator > (const T &p1, const T &p2) {
    return length(p1) > length(p2);
}

std::ostream& operator << (std::ostream& output_stream, const geometry_msgs::Point &p) {
    return output_stream << "[ " << p.x << ", " << p.y << ", " << p.z << " ]";
}

} // namespace geometry_msgs

#endif //DESPOT_HELPER_FUNCTIONS_H
