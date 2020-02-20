#ifndef __COMMON__
#define __COMMON__

#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI 3.14159265

using namespace Eigen;
using namespace std;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

#endif