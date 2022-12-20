#include "lasercalibration.h"

Vector3d calc_SVD(MatrixXd g_vec){
  char buffer[150];
  Vector3d normal;
  MatrixXd left_singular_mat;
  int U_cols;

  Serial.println("Generating Jacobi SVD obj");
  JacobiSVD<MatrixXd> svd(g_vec, ComputeThinU | ComputeThinV);
  
  sprintf(buffer, "Its left singular vectors are the columns of the thin U matrix:", svd.matrixU());
  Serial.println(buffer);

  Serial.println("Calculating left singular matrix");
  left_singular_mat = svd.matrixU();
  Serial.println("Setting normal to the 3 values");
  U_cols = left_singular_mat.cols();
  normal << left_singular_mat(0,U_cols-1), left_singular_mat(1,U_cols-1), left_singular_mat(2,U_cols-1);

  Serial.print(normal(0));
  Serial.print(", ");
  Serial.print(normal(1));
  Serial.print(", ");
  Serial.print(normal(2));
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
