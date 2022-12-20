#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include<iostream>

using namespace Eigen;
using std::cout;

Matrix3cd calc_magnetometer_HSI(Matrix3Xd meas_mat);