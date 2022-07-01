#include <MatrixMath.h>
#include <Math.h>


float V_exp[1], V_pred[1], V_ocv;
float del_t = 100;

float x_pred[3][1], P_pred[3][3], L[3][3], L_cpy[3][3], P_predC_trans[3][1];
float temp1[3][3], temp3[3][1], temp4[1][1];

float  SoC = 1;
float  V_ct = 0.007;
float  V_df = 0.004;
float  tau_ct = 4;
float  tau_df = 100;
float  R_ct = 0.001;
float  R_df = 0.01;
float  R_0 = 0.2;
float  R_l = 10;
float  Charge = 0.6*3600;
  
float I = 0;

float x[3][1] = {{SoC},
                 {V_ct},
                 {V_df}};
                
float A[3][3] = {{1, 0, 0},
                 {0, exp(-(del_t/1000)/tau_ct), 0},
                 {0, 0, exp(-(del_t/1000)/tau_df)}};
                
float B[3][1] = {{-(del_t/1000)/Charge},
                 {R_ct*(1-exp(-(del_t/1000)/tau_ct))},
                 {R_df*(1-exp(-(del_t/1000)/tau_df))}};

float P[3][3] = {{1e-3, 0, 0},
                 {0, 1e-6, 0},
                 {0, 0, 1e-6}};

float R[1] = {1e-3};

float Q[3][3] = {{0, 0, 0},
                 {0, 1e-4, 0},
                 {0, 0, 1e-5}};

float C[1][3] = {1, -1, -1};

float C_trans[3][1] = {{ 0},
                       {-1},
                       {-1}};

float Id[3][3] = {{1, 0, 0},
                  {0, 1, 0},
                  {0, 0, 1}};

void setup() {
  Serial.begin(9600);
 
  pinMode(2, OUTPUT);
}

void loop() {
  //V and I measurement
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
  
  //Extended Kalman Estimator

  /* Algorithms based on 
   * Carlo Taborelli, Simona Onori. "State of Charge Estimation
   * Using Extended Kalman Filters for Battery Management System".
   */

  //19a
  Matrix.Multiply((mtx_type*)A, (mtx_type*)x, 3, 3, 1, (mtx_type*)temp3); 
  Matrix.Scale((mtx_type*)B, 3, 1, (mtx_type)I);
  Matrix.Add((mtx_type*)temp3, (mtx_type*)B, 3, 1, (mtx_type*)x_pred);

  //19b
  Matrix.Multiply((mtx_type*)A, (mtx_type*)P, 3, 3, 3, (mtx_type*)temp1);
  Matrix.Multiply((mtx_type*)temp1, (mtx_type*)A, 3, 3, 3, (mtx_type*)temp1);
  Matrix.Add((mtx_type*)temp1, (mtx_type*)Q, 3, 1, (mtx_type*)P_pred); 

  /*Parameter update
   * Values based on 
   * yu, Zhihao & Huai, Ruituo & Xiao, Linjing. (2015). 
   * "State-of-Charge Estimation for Lithium-Ion Batteries 
   * Using a Kalman Filter Based on Local Linearization".
   */
  C[0][0] = (3149.3 - (288.24/x_pred[0][0]) - (0.5748*x_pred[0][0]) + (54.8063*log(x_pred[0][0]))- (29.833*log(1-x_pred[0][0])))/(2*x_pred[0][0]);
  
  //20a 
  Matrix.Multiply((mtx_type*)P_pred, (mtx_type*)C_trans, 3, 3, 1, (mtx_type*)P_predC_trans);
  Matrix.Multiply((mtx_type*)C, (mtx_type*)P_predC_trans, 1, 3, 1, (mtx_type*)temp4);
  Matrix.Add((mtx_type*)R, (mtx_type*)temp4, 1, 1, (mtx_type*)temp4); 
  Matrix.Invert((mtx_type*)temp4, 1);
  Matrix.Scale((mtx_type*)P_predC_trans, 3, 1, (mtx_type)temp4[0][0]);
  Matrix.Copy((mtx_type*)P_predC_trans, 3, 1, (mtx_type*)L);
  Matrix.Copy((mtx_type*)L, 3, 1, (mtx_type*)L_cpy);

  //20b
  float aux[1] = {(R_0*I)};
  Matrix.Multiply((mtx_type*)C, (mtx_type*)x_pred, 1, 3, 1, (mtx_type*)V_pred);
  Matrix.Subtract((mtx_type*)V_pred, (mtx_type*)aux, 1, 1, (mtx_type*)V_pred);
  Matrix.Subtract((mtx_type*)V_exp, (mtx_type*)V_pred, 1, 1,(mtx_type*) temp4);
  Matrix.Scale((mtx_type*)L_cpy, 3, 1, (mtx_type)temp4[0][0]);
  Matrix.Add((mtx_type*)x_pred, (mtx_type*)L_cpy, 3, 1, (mtx_type*)x);

  //20c
  Matrix.Multiply((mtx_type*)L, (mtx_type*)C, 3, 1, 3, (mtx_type*)temp1);
  Matrix.Subtract((mtx_type*)Id, (mtx_type*)temp1, 3, 3, (mtx_type*)temp1);
  Matrix.Multiply((mtx_type*)temp1, (mtx_type*)P_pred, 3, 3, 3, (mtx_type*)P);
  
  //Output
  Serial.println(x[0][0]);
  Serial.println(I);
  Serial.println(V_ocv);


}
