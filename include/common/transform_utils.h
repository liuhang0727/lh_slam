#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include "common_headers.h"

inline Eigen::Isometry3f getTransformYXZT(float tx, float ty, float tz,
                                          float roll, float pitch, float yaw)
{
    // Roll pitch and yaw in Radians
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
    t.translate(Eigen::Vector3f(tx, ty, tz));  //translate
    t.prerotate(q);  //prerotate
    return t;
}


#endif // TRANSFORM_UTILS_H