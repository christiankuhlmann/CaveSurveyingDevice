#include "magnetometer.h"

Matrix3cd calc_magnetometer_HSI(Matrix3Xd meas_mat){
  MatrixXd centered = meas_mat.colwise() - meas_mat.rowwise().mean();
  Matrix3d cov = (centered.transpose() * centered) / double(meas_mat.cols() - 1);
  EigenSolver<Matrix3d> eig;
  eig.compute(cov);
  cout << eig.eigenvalues() << "\n";
  Vector3cd vec = eig.eigenvalues();
  Vector3cd eigenvalues_sqrt = vec.array().pow(0.5);
  DiagonalMatrix<std::complex<double>, 3> diag_sqrt_eig(eigenvalues_sqrt);
  Matrix3cd T = eig.eigenvectors() * diag_sqrt_eig;
  Matrix3cd inv_T = T.inverse();
  return inv_T;
}