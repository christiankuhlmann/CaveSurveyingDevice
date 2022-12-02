#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

// #include <Eigen30.h>
// #include <EigenAVR.h>


using namespace Eigen;

const int LIDAR_CALIB_ROT_N = 8;
const int DIMS = 3;

void setup() {
  // put your setup code here, to run once:

}

void loop(){
  Serial.begin(9600);
}

void init(){
  Matrix<double,1,3> LIDAR_vec[3];
  Matrix<double,1,3> magnetometer_compensation[3];
}

//Matrix<double,3,1> calc_tilt_vec(Matrix<double,LIDAR_CALIB_ROT_N,3> g_vec){
void calc_tilt_vec(Matrix<double,LIDAR_CALIB_ROT_N,3> g_vec){
  // Initialise SVD object with given length to remove need for dynamic memory allocation
  JacobiSVD<MatrixXd, ComputeThinU> svd(g_vec);
  // Get first left singular value
  Matrix<double, 3, LIDAR_CALIB_ROT_N> u_vec = svd.matrixU();

  // Matrix<double, 3, 1> tilt_vec;
  // tilt_vec << u_vec(0,0),
  //             u_vec(1,0),
  //             u_vec(2,0);

  Serial.print("Left-most singular value:");
  Serial.print(u_vec(0,0));
  Serial.print(", ");
  Serial.print(u_vec(1,0));
  Serial.print(", ");
  Serial.print(u_vec(2,0));
  // Serial.print(tilt_vec(0,0))
  //return tilt_vec;
}


