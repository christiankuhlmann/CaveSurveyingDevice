#include <Arduino.h>
#include <Adafruit_BNO055.h>

#include <iostream>
#include <math.h>
#include <nvs_flash.h>

#include "filefuncs.h"
#include "lasercalibration.h"
#include "magnetometer.h"

// Defining global variables
static Adafruit_BNO055 bno;
static float tilt_arr[3];
static float orientation_arr[3];


void init_bno(){
  // COPIED CODE
  bno = Adafruit_BNO055(55);
  if (!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void get_bno(){
  sensors_event_t event; 
  bno.getEvent(&event);
  orientation_arr[0] = event.orientation.x;
  orientation_arr[1] = event.orientation.y;
  orientation_arr[2] = event.orientation.z;
}


void setup(){
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Beginning!");
  MatrixXd g_vec(3,9);
  g_vec <<  -8.75384669e-01, 2.83071261e-01, 3.54998692e-01, 2.83071261e-01, 1.09423084e-01,-6.42250940e-02,-1.36152524e-01,-6.42250940e-02, 1.09423084e-01,
            -4.93432455e-17,-1.00000000e+00,-4.93432455e-17, 1.00000000e+00, 1.41421356e+00, 1.00000000e+00, 1.72701359e-16,-1.00000000e+00,-1.41421356e+00,
             1.54353936e-01, 9.65513511e-01, 1.37343424e+00, 9.65513511e-01,-1.92942420e-02,-1.00410199e+00,-1.41202272e+00,-1.00410199e+00,-1.92942420e-02;
  // g_vec << 1.0,2.0,3.0,
  //           4.0,5.0,6.0,
  //           7.0,8.0,9.0;
  Serial.println("SVD time!");
  calc_SVD(g_vec);
}

void loop(){
  Serial.println("Done!");
  while(true);
}