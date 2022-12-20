#ifndef HEADER_lasercalibration
#define HEADER_lasercalibration


#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include <iostream>

using namespace Eigen;
using std::cout;

Vector3d calc_SVD(MatrixXd g_vec);
Matrix<double,3,1> calc_true_vec(Vector3d normal_vec, VectorXd laser_distances, double disto_len);

#endif