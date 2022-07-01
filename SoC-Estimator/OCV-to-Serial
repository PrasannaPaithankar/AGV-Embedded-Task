#include <MatrixMath.h>
#include <Math.h>


float V_ocv;
float del_t = 100;

float  SoC = 1;
float  R_l = 2.2;
float  Charge = 0.6*3600;
  
float I = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {

  //Open circuit voltage measurement
  V_ocv = analogRead(A0);
  V_ocv = V_ocv/1023;
  V_ocv = V_ocv*5;
  
  //Output (CSV Readable)
  Serial.print(V_ocv);
}
