#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include <iostream>
#include <math.h>


using namespace Eigen;
using std::cout;

Vector3d calc_SVD(Matrix3Xd g_vec){
  Vector3d normal;
  Eigen::MatrixXd left_singular_mat;
  Eigen::JacobiSVD<MatrixXd> svd(g_vec);
  cout << "Its left singular vectors are the columns of the thin U matrix:" << svd.matrixU();
  left_singular_mat = svd.matrixU();
  normal << (left_singular_mat(0,0), left_singular_mat(1,0), left_singular_mat(2,0));

  Serial.print(normal(0,0));
  Serial.print(", ");
  Serial.print(normal(0,1));
  Serial.print(", ");
  Serial.print(normal(0,2));
  return normal;
};


Matrix<double,3,1> calc_true_vec(Vector3d normal_vec, VectorXd laser_distances, double disto_len){
  double laser_len = laser_distances.mean();
  Matrix<double,3,1> x_axis;

  x_axis << (1.0,0.0,0.0);

  double alpha = acos(normal_vec.dot(x_axis) / (normal_vec.norm()*x_axis.norm()));
  if (alpha > M_PI_2){
    alpha = M_PI-alpha;
    normal_vec = -normal_vec;
  };

  double l_0 = disto_len;
  double l_1 = laser_len;
  double l_2 = l_0 * sin(alpha);
  double l_3 = l_0 * cos(alpha) + sqrt(pow(l_1,2)-pow(l_2,2));

  Vector3d true_vec = l_3 * x_axis + l_0 * normal_vec;
  return true_vec;
}

Matrix3cd calc_magnetometer_HSI(Matrix3Xd meas_mat){
  MatrixXd centered = meas_mat.colwise() - meas_mat.rowwise().mean();
  Matrix3d cov = (centered.transpose() * centered) / double(meas_mat.cols() - 1);
  Eigen::EigenSolver<Matrix3d> eig;
  eig.compute(cov);
  cout << eig.eigenvalues() << "\n";
  Eigen::Vector3cd vec = eig.eigenvalues();
  Eigen::Vector3cd eigenvalues_sqrt = vec.array().pow(0.5);
  DiagonalMatrix<std::complex<double>, 3> diag_sqrt_eig(eigenvalues_sqrt);
  Matrix3cd T = eig.eigenvectors() * diag_sqrt_eig;
  Matrix3cd inv_T = T.inverse();
  return inv_T;
}



void setup(){
  Matrix3Xd g_vec;
  g_vec << (1,2,3,4,5,6,7,8,9,10,11,12);
  calc_SVD(g_vec);
}

void loop(){

}