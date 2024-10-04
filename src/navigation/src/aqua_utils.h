#ifndef AQUA_UTILS
#define AQUA_UTILS

#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

const double G = 9.807;

std::vector<double>
calculateFreeAcceleration(const std::vector<double> &accImu,
                           const tf2::Quaternion &q);

std::vector<double> Quaternion2RPY(const tf2::Quaternion &q);
#endif