#include <MatrixMath.h>
#include <Math.h>


float V_exp[1], V_ocv;
float del_t = 100;

float  SoC = 1;
float  R_l = 2.2;
float  Charge = 0.6*3600;
  
float I = 0;

void setup() {
  Serial.begin(9600);

  //Transistor Base pin
  pinMode(2, OUTPUT);
}

void loop() {
  //V (across bleeding resistor) and I measurement
  digitalWrite(2, HIGH);
  V_exp[0] = analogRead(A1);
  V_exp[0] = V_exp[0]/1023;
  V_exp[0] = V_exp[0]*5;
  I = V_exp[0]/R_l;
  delay(del_t);
  digitalWrite(2, LOW);

  //Open circuit voltage measurement
  V_ocv = analogRead(A0);
  V_ocv = V_ocv/1023;
  V_ocv = V_ocv*5;
  
  //Output (CSV Readable)
  Serial.print(V_ocv);
  Serial.print(",");
  Serial.println(I);
}
