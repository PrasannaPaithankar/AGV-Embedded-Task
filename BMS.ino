#include <MatrixMath.h>
#include <Math.h>

float SoC, V_ct, V_df;
float V_exp[1], V_pred[1], V_ocv;
float del_t = 10;
float tau_ct , tau_df, R_ct, R_df, R_0;
float R_l;
float Charge;
float x_pred[3][1], P_pred[3][3], L[3][3], P_predC_trans[3][1];
mtx_type* temp1;

float I = 0;

float x[3][1] = {{SoC},
                {V_ct},
                {V_df}};
                
float A[3][3] = {{1, 0, 0},
                 {0, exp(-del_t/tau_ct), 0},
                 {0, 0, exp(-del_t/tau_df)}};
                
float B[3][1] = {{-del_t/Charge},
                 {R_ct*(1-exp(-del_t/tau_ct))},
                 {R_df*(1-exp(-del_t/tau_df))}};

float P[3][3] = {{1e-3, 0, 0},
                 {0, 1e-6, 0},
                 {0, 0, 1e-6}};

float R[1] = {1e-3};

float Q[3][3] = {{1, 0, 0},
                 {0, 1e-4, 0},
                 {0, 0, 1e-4}};

float C[1][3] = {1, -1, -1};

float C_trans[3][1] = {{ 1},
                       {-1},
                       {-1}};

float Id[3][3] = {{1, 0, 0},
                  {0, 1, 0},
                  {0, 0, 1}};

void setup() {
  Serial.begin(9600);
  
  SoC = 1;
  V_ct = 0;
  V_df = 0;
  tau_ct = 0;
  tau_df = 0;
  R_ct = 0;
  R_df = 0;
  R_0 = 0;
  R_l = 0;
  Charge = 0.6;
 
  pinMode(2, OUTPUT);
}

void loop() {
  //V and I measurement
  digitalWrite(2, HIGH);
  V_exp[0] = 5*analogRead(A0)/1023;
  I = R_l*V_exp[0];
  delay(del_t);
  digitalWrite(2, LOW);

  //Open circuit voltage measurement
  V_ocv = 5*analogRead(A1)/1023;

  //Extended Kalman Estimator

  /* Algorithms based on
   * Carlo Taborelli, Simona Onori. "State of Charge Estimation
   * Using Extended Kalman Filters for Battery Management System".
   */
   
  //19a
  Matrix.Multiply((mtx_type*)A, (mtx_type*)x, 3, 3, 1, (mtx_type*)temp1);
  Matrix.Scale((mtx_type*)B, 3, 1, (mtx_type)I);
  Matrix.Add((mtx_type*)temp1, (mtx_type*)B, 3, 1, (mtx_type*)x_pred);
  
  //19b
  Matrix.Multiply((mtx_type*)A, (mtx_type*)P, 3, 3, 3, (mtx_type*)temp1);
  Matrix.Multiply((mtx_type*)temp1, (mtx_type*)A, 3, 3, 3, (mtx_type*)temp1);
  Matrix.Add((mtx_type*)temp1, (mtx_type*)Q, 3, 1, (mtx_type*)P_pred); 

  //20a 
  Matrix.Multiply((mtx_type*)P_pred, (mtx_type*)C_trans, 3, 3, 1, (mtx_type*)P_predC_trans);
  Matrix.Multiply((mtx_type*)C, (mtx_type*)P_predC_trans, 1, 3, 1, (mtx_type*)temp1);
  Matrix.Add((mtx_type*)R, temp1, 1, 1, temp1); 
  Matrix.Invert(temp1, 1);
  Matrix.Scale(temp1, 3, 1, (mtx_type)temp1[0]);

  //20b
  float aux[1] = {(R_0*I)};
  Matrix.Multiply((mtx_type*)C, (mtx_type*)x_pred, 1, 3, 1, (mtx_type*)V_pred);
  Matrix.Subtract((mtx_type*)V_pred, (mtx_type*)aux, 1, 1, (mtx_type*)V_pred);
  Matrix.Subtract((mtx_type*)V_exp, (mtx_type*)V_pred, 1, 1,(mtx_type*) temp1);
  Matrix.Scale((mtx_type*)L, 3, 1, (mtx_type)temp1[0]);
  Matrix.Add((mtx_type*)x_pred, (mtx_type*)temp1, 3, 1, (mtx_type*)x);

  //20c
  Matrix.Multiply((mtx_type*)L, (mtx_type*)C, 3, 1, 3, (mtx_type*)temp1);
  Matrix.Subtract((mtx_type*)Id, temp1, 3, 3, temp1);
  Matrix.Multiply(temp1, (mtx_type*)P_pred, 3, 3, 3, (mtx_type*)P);

  //Output
  Serial.println(x[0][0]);
  Serial.println(V_ocv);


}
