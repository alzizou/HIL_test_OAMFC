#include <Wire.h>

const int n = 12;
const float k1 = 1e-0;
const float k2 = 1e-0;
const float B[n] = {1,1,1,1,1,1,1,1,1,1,1,1};
const float R[n] = {1,1,1,1,1,1,1,1,1,1,1,1};
const float Q[n] = {1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2,1e-2};
float P[n] = {1,1,1,1,1,1,1,1,1,1,1,1};
float A[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float g[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float x[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float xm[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float Dxm[3] = {0,0,0};
float yd[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float D_yd[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float e[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float zeta[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float sigma[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float u[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
const float Gamma1[n] = {1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-1,1e+1,1e+1,1e+1};
const float Gamma2[n] = {1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-2,1e-2,1e-2,1e-2};
const float rho1 = 1e-3;
const float rho2 = 1e-3;

float P_sigma[n] = {0,0,0,0,0,0,0,0,0,0,0,0};

float U_EM1_1 = 0;
float U_EM1_2 = 0;
float U_EM1_3 = 0;
float U_EM1_4 = 0;

float D_zeta[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float D_g[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float D_A[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float D_P[n] = {0,0,0,0,0,0,0,0,0,0,0,0};

float X_Ref = 0;
float Y_Ref = 0;
float Z_Ref = 0;

float ydm[3] = {X_Ref,Y_Ref,Z_Ref};

float Diff_err[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float ABS_Diff_err[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float SGN_Diff_err[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float omega[n] = {X_Ref,Y_Ref,Z_Ref,0,0,0,0,0,0,0,0,0};
float tau[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float eta[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float D_omega[n] = {0,0,0,0,0,0,0,0,0,0,0,0};
float D_tau[n] = {0,0,0,0,0,0,0,0,0,0,0,0};

float psi_d = 0;
float phi_d = 0;
float theta_d = 0;
float phi_d0 = 0;
float phi_d1 = 0;
float theta_d0 = 0;
float theta_d1 = 0;

unsigned long time1 = 0;
unsigned long time2 = 0;
float dt = 0.1;
float t = 0;

float U_Y[4] = {0,0,0,0};
float U_EM1 = 0;
float U_EM2 = 0;
float U_EM3 = 0;
float U_EM4 = 0;

float Kf = 1e-5;
float Beta = 1;
float L = 1;
float RM1_1[4] = {(1/(4*Kf)),1e-12,(-1/(2*L*Kf)),(1/(4*Beta*Kf))};
float RM1_2[4] = {(1/(4*Kf)),(1/(2*L*Kf)),1e-12,(-1/(4*Beta*Kf))};
float RM1_3[4] = {(1/(4*Kf)),1e-12,(1/(2*L*Kf)),(1/(4*Beta*Kf))};
float RM1_4[4] = {(1/(4*Kf)),(-1/(2*L*Kf)),1e-12,(-1/(4*Beta*Kf))};

int num = 0;

float E1 = 0;
float E2 = 0;
float E3 = 0;
float E4 = 0;

int32_t E1_Array[4];
int32_t E2_Array[4];
int32_t E3_Array[4];
int32_t E4_Array[4];

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
float DPo[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0} 
};
float Po[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0} 
};
float Qo_val = 10;
float Ro_val = 0.1;
float xm_hat[12][1] = {0,0,0,0,0,0,0,0,0,0,0,0};
float Ko[6][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float Ko1[6][3] = {
  {(1.0f)/Ro_val,0,0},
  {0,(1.0f)/Ro_val,0},
  {0,0,(1.0f)/Ro_val},
  {0,0,0},
  {0,0,0},
  {0,0,0}  
};
float DPo_term1[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0} 
};
float DPo_term2[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0} 
};
float DPo_term3[6][6] = {
  {Qo_val,0,0,0,0,0},
  {0,Qo_val,0,0,0,0},
  {0,0,Qo_val,0,0,0},
  {0,0,0,Qo_val,0,0},
  {0,0,0,0,Qo_val,0},  
  {0,0,0,0,0,Qo_val}
};
float DPo_term4[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0} 
};
float DPo_term41[6][6] = {
  {(1.0f)/Ro_val,0,0},
  {0,(1.0f)/Ro_val,0},
  {0,0,(1.0f)/Ro_val},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0}    
};
float DPo_term42[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0} 
};
float Dxm_hat[6][1] = {0,0,0,0,0,0};
float Dxm_hat_term12[6][1] = {0,0,0,0,0,0};
float Dxm_hat_term3[6][1] = {0,0,0,0,0,0};
float Dxm_hat_term31[6][1] = {0,0,0,0,0,0};
float zo[3][1] = {0,0,0};
float Udo[3][1] = {0,0,0};
float DPo_time[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0} 
};

float Rq_1[3] = {0,0,0};
float Rq_2[3] = {0,0,0};
float Rq_3[3] = {0,0,0};

float phi = 0;
float theta = 0;
float psi = 0;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);

  //Generating the reference signal
  ydm[0] = X_Ref;
  ydm[1] = Y_Ref;
  ydm[2] = Z_Ref;  
}

void loop() {
//Generating the sampling time
  time2 = millis();
  dt = (time2-time1)*1e-3;
  time1 = time2;      

  Wire.requestFrom(10,60);
  while(Wire.available()) {
    // Generating the measured signals.
    xm[0] = Ali_Read();
    xm[1] = Ali_Read();
    xm[2] = Ali_Read();
    xm[3] = Ali_Read();
    xm[4] = Ali_Read();
    xm[5] = Ali_Read();    
    xm[6] = Ali_Read();
    xm[7] = Ali_Read();
    xm[8] = Ali_Read();
    xm[9] = Ali_Read();
    xm[10] = Ali_Read();
    xm[11] = Ali_Read();    

    Dxm[0] = Ali_Read();
    Dxm[1] = Ali_Read();
    Dxm[2] = Ali_Read();
  }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Kalman-Filter
  phi = xm_hat[3][0];
  theta = xm_hat[4][0];
  psi = xm_hat[5][0];      

  Rq_1[0] = cos(psi) * cos(theta);
  Rq_1[1] = (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi));
  Rq_1[2] = (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi));
  Rq_2[0] = sin(psi) * cos(theta);
  Rq_2[1] = (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi));
  Rq_2[2] = (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi));
  Rq_3[0] = -sin(theta);
  Rq_3[1] = cos(theta) * sin(phi);
  Rq_3[2] = cos(theta) * cos(phi); 
  
  for(int r=0;r<3;r++){
    zo[r][0] = xm[r] + random(-0.5,0.5);   
    Udo[r][0] = Dxm[r] + 0.1*random(-0.5,0.5);   
  };

  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      DPo_term1[i][j] = Po[3+i][j];      
      DPo_term1[i][3+j] = Po[3+i][3+j];
      DPo_term2[i][j] = Po[i][3+j];
      DPo_term2[3+i][j] = Po[3+i][3+j];
    };
  };
   
  MatrixMultiplyAli(*Po,*DPo_term41,6,6,6,*DPo_term42);
  MatrixMultiplyAli(*DPo_term42,*Po,6,6,6,*DPo_term4);   
  MatrixMultiplyAli(*Po,*Ko1,6,6,3,*Ko);

  Dxm_hat_term12[0][0] = xm_hat[6][0];
  Dxm_hat_term12[1][0] = xm_hat[7][0];
  Dxm_hat_term12[2][0] = xm_hat[8][0];
  Dxm_hat_term12[3][0] = Rq_1[0]*Udo[0][0] + Rq_1[1]*Udo[1][0] + Rq_1[2]*Udo[2][0];  
  Dxm_hat_term12[4][0] = Rq_2[0]*Udo[0][0] + Rq_2[1]*Udo[1][0] + Rq_2[2]*Udo[2][0];  
  Dxm_hat_term12[5][0] = Rq_3[0]*Udo[0][0] + Rq_3[1]*Udo[1][0] + Rq_3[2]*Udo[2][0];  

  Dxm_hat_term31[0][0] = zo[0][0] - xm_hat[0][0];
  Dxm_hat_term31[1][0] = zo[1][0] - xm_hat[1][0];
  Dxm_hat_term31[2][0] = zo[2][0] - xm_hat[2][0];
     
  MatrixMultiplyAli(*Ko,*Dxm_hat_term31,6,3,1,*Dxm_hat_term3);

  for(int i1=0;i1<6;i1++){
    Dxm_hat[i1][0] = Dxm_hat_term12[i1][0] + Dxm_hat_term3[i1][0];
    for(int j1=0;j1<6;j1++){
      DPo[i1][j1] = DPo_term1[i1][j1] + DPo_term2[i1][j1] + DPo_term3[i1][j1] - DPo_term4[i1][j1];
      Po[i1][j1] = Po[i1][j1] + DPo[i1][j1]*dt;
    }
  }

  xm_hat[0][0] = xm_hat[0][0] + Dxm_hat[0][0]*dt;
  xm_hat[1][0] = xm_hat[1][0] + Dxm_hat[1][0]*dt;
  xm_hat[2][0] = xm_hat[2][0] + Dxm_hat[2][0]*dt;
  xm_hat[6][0] = xm_hat[6][0] + Dxm_hat[3][0]*dt;
  xm_hat[7][0] = xm_hat[7][0] + Dxm_hat[4][0]*dt;
  xm_hat[8][0] = xm_hat[8][0] + Dxm_hat[5][0]*dt;
  xm_hat[3][0] = xm[3];
  xm_hat[4][0] = xm[4];
  xm_hat[5][0] = xm[5];
  xm_hat[9][0] = xm[9];
  xm_hat[10][0] = xm[10];
  xm_hat[11][0] = xm[11];

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//Generating the reference signal
  t = t + dt;
  if (t > 20) {
    ydm[0] =  2*cos(0.2*(t-20)) - 2; 
    ydm[1] = 2*sin(0.2*(t-20));      
  }; 
  ydm[2] = 0.2*t;    

  phi_d = asin(min(max((-u[7]/(U_Y[0]+1e-3)),-1),1));      
  theta_d = atan(u[6]/(u[8] + 1e-3));   
      
  yd[0] = ydm[0];
  yd[1] = ydm[1];
  yd[2] = ydm[2];
  yd[3] = phi_d;
  yd[4] = theta_d;
  yd[5] = psi_d;
  yd[6] = u[0];
  yd[7] = u[1];
  yd[8] = u[2];
  yd[9] = u[3];
  yd[10] = u[4];
  yd[11] = u[5]; 

// Main loop. generating the control signals. 
  num = 0; 
  e[num] = yd[num] - xm_hat[num][0];    
  zeta[num] = zeta[num] + D_zeta[num]*dt;
  sigma[num] = e[num] + zeta[num];                                        
  g[num] = g[num] + D_g[num]*dt;
  A[num] = A[num] + D_A[num]*dt;                                       
  P[num] = P[num] + D_P[num]*dt;    
  P_sigma[num] = P[num]*sigma[num];                                                                 
  u[num] = D_yd[num] - (A[num]*xm_hat[num][0]) - g[num] - zeta[num] + (((1.0f)+(((2.0f)/P[num])*Q[num])+A[num])*sigma[num]) - 0.25*P_sigma[num]; //(1/B[num])* //*R[num]*B[num]    
  D_zeta[num] = e[num];
  D_g[num] = -Gamma1[num]*P_sigma[num] - rho1*Gamma1[num]*g[num];
  D_A[num] = -Gamma2[num]*P_sigma[num]*(xm_hat[num][0]-sigma[num]) - rho2*Gamma2[num]*A[num];
  D_P[num] = (2.0f)*A[num]*P[num] - P[num]*P[num] + (2.0f)*Q[num]; //*B[num]*R[num]*B[num] //+ P[num]*A[num]

  num = num + 1;
  for(num;num<n;num++) {
    e[num] = yd[num] - xm_hat[num][0];    
    zeta[num] = zeta[num] + D_zeta[num]*dt;
    sigma[num] = e[num] + zeta[num];                                        
    g[num] = g[num] + D_g[num]*dt;
    A[num] = A[num] + D_A[num]*dt;                                       
    P[num] = P[num] + D_P[num]*dt;   
    P_sigma[num] = P[num]*sigma[num];                                                                   
    u[num] = D_yd[num] - (A[num]*xm_hat[num][0]) - g[num] - zeta[num] + (((1.0f)+(((2.0f)/P[num])*Q[num])+A[num])*sigma[num]) - 0.25*P_sigma[num];   //(1/B[num])*  //*R[num]*B[num]
    D_zeta[num] = e[num];
    D_g[num] = -Gamma1[num]*P_sigma[num] - rho1*Gamma1[num]*g[num];
    D_A[num] = -Gamma2[num]*P_sigma[num]*(xm_hat[num][0]-sigma[num]) - rho2*Gamma2[num]*A[num];
    D_P[num] = (2.0f)*A[num]*P[num] - P[num]*P[num] + (2.0f)*Q[num];   //*B[num]*R[num]*B[num]   //+ P[num]*A[num]
  }

// Generating the differentiation of augmented reference signal   
  for (int ii=0;ii<n;ii++) {      
    Diff_err[ii] = omega[ii] - yd[ii];           
    if (Diff_err[ii] > 0) {  
      ABS_Diff_err[ii] = Diff_err[ii];                   
    }else  {                              
      ABS_Diff_err[ii] = -Diff_err[ii];            
    }
    SGN_Diff_err[ii] = Diff_err[ii] / (ABS_Diff_err[ii] + 1e-2);
    tau[ii] = tau[ii] + D_tau[ii]*dt;
    eta[ii] = tau[ii] - k1*sqrt(ABS_Diff_err[ii])*SGN_Diff_err[ii];    
    omega[ii] = omega[ii] + D_omega[ii]*dt;               
    D_tau[ii] = -k2*SGN_Diff_err[ii];  
    D_yd[ii] = eta[ii];   
    D_omega[ii] = eta[ii];  
  }

  U_Y[0] = sqrt(u[6]*u[6] + u[7]*u[7] + u[8]*u[8]);
  U_Y[1] = u[9];
  U_Y[2] = u[10];
  U_Y[3] = u[11];    
  U_EM1 = sqrt(abs(Ali_Product(RM1_1, U_Y, 4)));
  U_EM2 = sqrt(abs(Ali_Product(RM1_2, U_Y, 4)));
  U_EM3 = sqrt(abs(Ali_Product(RM1_3, U_Y, 4)));
  U_EM4 = sqrt(abs(Ali_Product(RM1_4, U_Y, 4)));
  
  E1 = U_EM1 * 1000L;
  E2 = U_EM2 * 1000L;
  E3 = U_EM3 * 1000L;
  E4 = U_EM4 * 1000L;
  Ali_I2C_Preparation(E1,E1_Array);
  Ali_I2C_Preparation(E2,E2_Array);
  Ali_I2C_Preparation(E3,E3_Array);
  Ali_I2C_Preparation(E4,E4_Array);
  Wire.beginTransmission(10);
  Ali_Send(E1_Array);
  Ali_Send(E2_Array);
  Ali_Send(E3_Array);
  Ali_Send(E4_Array);
  Wire.endTransmission();

  Serial.print("Time: ");
  Serial.println(t,4);
//  Serial.print("Yd_x: ");
//  Serial.println(ydm[0],4);
//  Serial.print("Yd_y: ");
//  Serial.println(ydm[1],4);
//  Serial.print("Yd_z: ");
//  Serial.println(ydm[2],4);
//  Serial.print("Err_x: ");
//  Serial.println(e[0],4);
//  Serial.print("Err_y: ");
//  Serial.println(e[1],4);
//  Serial.print("Err_z: ");
//  Serial.println(e[2],4);
//  Serial.print("g1: ");
//  Serial.println(g[0],4);
//  Serial.print("g2: ");
//  Serial.println(g[1],4);
//  Serial.print("g3: ");
//  Serial.println(g[2],4);
//  Serial.print("g4: ");
//  Serial.println(g[3],4);
//  Serial.print("g5: ");
//  Serial.println(g[4],4);
//  Serial.print("g6: ");
//  Serial.println(g[5],4);
//  Serial.print("g7: ");
//  Serial.println(g[6],4);
//  Serial.print("g8: ");
//  Serial.println(g[7],4);
//  Serial.print("g9: ");
//  Serial.println(g[8],4);
//  Serial.print("g10: ");
//  Serial.println(g[9],4);
//  Serial.print("g11: ");
//  Serial.println(g[10],4);
//  Serial.print("g12: ");
//  Serial.println(g[11],4);
//  Serial.print("A7: ");
//  Serial.println(A[6],4);
//  Serial.print("A8: ");
//  Serial.println(A[7],4);
//  Serial.print("A9: ");
//  Serial.println(A[8],4);
//  Serial.print("A10: ");
//  Serial.println(A[9],4);
//  Serial.print("A11: ");
//  Serial.println(A[10],4);
//  Serial.print("A12: ");
//  Serial.println(A[11],4);
//  Serial.print("P7: ");
//  Serial.println(P[6],4);
//  Serial.print("P8: ");
//  Serial.println(P[7],4);
//  Serial.print("P9: ");
//  Serial.println(P[8],4);
//  Serial.print("P10: ");
//  Serial.println(P[9],4);
//  Serial.print("P11: ");
//  Serial.println(P[10],4);
//  Serial.print("P12: ");
//  Serial.println(P[11],4);
//  Serial.print("U_E1: ");
//  Serial.println(U_EM1,4);
//  Serial.print("U_E2: ");
//  Serial.println(U_EM2,4);
//  Serial.print("U_E3: ");
//  Serial.println(U_EM3,4);
//  Serial.print("U_E4: ");
//  Serial.println(U_EM4,4);
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
float Ali_Read() {
    int8_t x11 = Wire.read();
    if (x11>127) {
      x11 = 256 - x11;
      x11 = x11 * -1;
    }
    uint8_t x12 = Wire.read();        
    uint8_t x21 = Wire.read();
    uint8_t x22 = Wire.read();
    int16_t x13 = x11;    
    x13 = (x13 << 8) | x12;
    uint16_t x23 = x21;
    x23 = (x23 << 8) | x22;
    int32_t x3 = (x13 << 8);     
    x3 = (x3 << 8) | x23;        
    float x4_L = (float)x3/1000; 
    return x4_L;
}

float Ali_Send(int32_t *inp) {
  Wire.write(inp[0]);
  Wire.write(inp[1]);
  Wire.write(inp[2]);
  Wire.write(inp[3]);
}

float Ali_I2C_Preparation(float inp, int32_t *out) {
  out[0] = (((int32_t)inp) >> 24) & 0XFF;
  out[1] = (((int32_t)inp) >> 16) & 0XFF;
  out[2] = (((int32_t)inp) >> 8) & 0XFF;
  out[3] = (((int32_t)inp) ) & 0XFF;
}

float Ali_Product(float *inp1, float *inp2, int na) {
  float out = 0;
  for (int i=0;(i<na);i++) {
    out = out + (inp1[i] * inp2[i]);    
  }
  return out;
}

float MatrixScalarMultiply(float *inp,int n_inp, int m_inp,float dt,float *out){
  for(int i=0;i<n_inp;i++){
    for(int j=0;j<m_inp;j++){
      out[m_inp*i + j] = dt * inp[m_inp*i + j];
    }
  }  
}

float MatrixMultiplyAli(float *inp1, float *inp2, int n_inp, int m_inp, int p_inp, float*out){
  for(int i=0;i<n_inp;i++){
    for(int j=0;j<p_inp;j++){
      out[i*p_inp + j] = 0;
      for(int k=0;k<m_inp;k++){
        out[i*p_inp + j] = out[i*p_inp + j] + (inp1[i*m_inp + k] * inp2[k*p_inp + j]);
      }
    }
  }  
}

