#ifndef DESPOT_ROS_UTILITIES_H
#define DESPOT_ROS_UTILITIES_H

#include <cmath>
#include "geometry_msgs/Vector3.h"

namespace ros_utils {

template <typename T>
T operator+(const T& vec1, const T& vec2)
{
    T vec;
    vec.x = vec1.x + vec2.x;
    vec.y = vec1.y + vec2.y;
    vec.z = vec1.z + vec2.z;
    return vec;
}

template <typename T>
T operator-(const T& vec1, const T& vec2)
{
    T vec;
    vec.x = vec1.x - vec2.x;
    vec.y = vec1.y - vec2.y;
    vec.z = vec1.z - vec2.z;
    return vec;
}

template <typename T>
double length(const T& vec)
{
    return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

template <typename T>
double distance(T from, T to)
{
    T sub_vec = to - from;
    return length(sub_vec);
}

template <typename T>
bool operator==(const T& v1, const T& v2)
{
    return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

template <typename T>
bool operator>(const T& v1, const T& v2)
{
    return length(v1) > length(v2);
}

template <typename T>
bool operator<(const T& v1, const T& v2)
{
    return length(v1) < length(v2);
}

template <typename T>
bool operator>=(const T& v1, const T& v2)
{
    return length(v1) >= length(v2);
}

template <typename T>
bool operator<=(const T& v1, const T& v2)
{
    return length(v1) <= length(v2);
}


} // namespace ros_utils

#endif //DESPOT_ROS_UTILITIES_H
