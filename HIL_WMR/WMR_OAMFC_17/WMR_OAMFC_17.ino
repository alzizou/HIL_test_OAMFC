#include <Wire.h>

const int n = 2;
const float Thresh = 1e-0;
const float Thresh_R = 1e-2;
const float Thresh_T = 1e-2;
const float k1 = 1e0;
const float k2 = 1e0;
const float B[n] = {1,1};
const float R[n] = {1,1};
const float Q[n] = {0.01,0.01};
float P_T[n] = {1,1};
float P_R[n] = {1,1};
float A_T[n] = {0,0};
float A_R[n] = {0,0};
float g_T[n] = {0,0};
float g_R[n] = {0,0};
float x_T[n] = {0,0};
float x_R[n] = {0,0};
float xm[5] = {0,0,0,0,0};
float Dxm = 0;
float yd_T[n] = {0,0};
float yd_R[n] = {0,0};
float Td = 0;
float D_yd_T[n] = {0,0};
float D_yd_R[n] = {0,0};
float ydm[2] = {0,0};
float D_ydm[2] = {0,0};
float e_T[n] = {0,0};
float e_R[n] = {0,0};
float zeta_T[n] = {0,0};
float zeta_R[n] = {0,0};
float sigma_T[n] = {0,0};
float sigma_R[n] = {0,0};
float u1_T[n] = {0,0};
float u1_R[n] = {0,0};
float u2_T[n] = {0,0};
float u2_R[n] = {0,0};
float u_T[n] = {0,0};
float u_R[n] = {0,0};
const float Gamma1_T[n] = {0.001,10};
const float Gamma2_T[n] = {0.001,0.1};
const float Gamma1_R[n] = {0.001,10};
const float Gamma2_R[n] = {0.001,0.1};
const float rho1 = 1e-3;
const float rho2 = 1e-3;

float D_zeta_T[n] = {0,0};
float D_zeta_R[n] = {0,0};
float D_g_T[n] = {0,0};
float D_g_R[n] = {0,0};
float D_A_T[n] = {0,0};
float D_A_R[n] = {0,0};
float D_P_T[n] = {0,0};
float D_P_R[n] = {0,0};

float X_Ref = 0;
float Y_Ref = 0;
float XS_Ref = 0;//Y_Ref / (X_Ref);
float Td_Ref = 0;//atan(XS_Ref);
float Posd_Ref = 0;//(X_Ref*cos(Td_Ref)) + (Y_Ref*sin(Td_Ref));

float err0 = 0;
float y_Err = 0;
float x_Err = 0;
float XS = 0;
float Pos = 0;
float Posd = 0;
float err = 0;

float Diff_err_T[n] = {0,0};
float Diff_err_R[n] = {0,0};
float SGN_Diff_err_T[n] = {0,0};
float SGN_Diff_err_R[n] = {0,0};
float ABS_Diff_err_T[n] = {0,0};
float ABS_Diff_err_R[n] = {0,0};
float omega_T[n] = {Posd_Ref,0};
float omega_R[n] = {Td_Ref,0};
float tau_T[n] = {0,0};
float tau_R[n] = {0,0};
float eta_T[n] = {0,0};
float eta_R[n] = {0,0};
float D_omega_T[n] = {0,0};
float D_omega_R[n] = {0,0};
float D_tau_T[n] = {0,0};
float D_tau_R[n] = {0,0};

float Diff_err_Ref[2] = {0,0};
float SGN_Diff_err_Ref[2] = {0,0};
float ABS_Diff_err_Ref[2] = {0,0};
float omega_Ref[2] = {0,0}; //{X_Ref,Y_Ref};
float tau_Ref[2] = {0,0};
float eta_Ref[2] = {0,0};
float D_omega_Ref[2] = {0,0};
float D_tau_Ref[2] = {0,0};

unsigned long time1 = 0;
unsigned long time2 = 0;
float dt = 0.1;
float t = 0;

float U_EM1 = 0;
float U_EM2 = 0;
float E1 = 0;
float E2 = 0;
int32_t E1_Array[4];
int32_t E2_Array[4];

float DPo[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float Po[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float Qo_val = 1;
float Ro_val = 1;
float xm_hat[5][1] = {0,0,0,0,0};
float Ao[3][3] = {
  {0,0,cos(xm_hat[2][0])},
  {0,0,sin(xm_hat[2][0])},
  {0,0,0}  
};
float Ko[3][2] = {
  {0,0},
  {0,0},
  {0,0}   
};
float Ko1[3][2] = {
  {(1.0f)/Ro_val,0},
  {0,(1.0f)/Ro_val},
  {0,0,} 
};
float DPo_term1[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float DPo_term2[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float DPo_term3[3][3] = {
  {Qo_val,0,0},
  {0,Qo_val,0},
  {0,0,Qo_val} 
};
float DPo_term4[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float DPo_term41[3][3] = {
  {(1.0f)/Ro_val,0,0},
  {0,(1.0f)/Ro_val,0},
  {0,0,0} 
};
float DPo_term42[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float Dxm_hat[3][1] = {0,0,0};
float Dxm_hat_term12[3][1] = {0,0,0};
float Dxm_hat_term3[3][1] = {0,0,0};
float Dxm_hat_term31[3][1] = {0,0,0};
float zo[2][1] = {0,0};
float Udo[1][1] = {0};
float Ao_trans[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};
float DPo_time[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};


void setup() {
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);

  MatrixTransposeAli(*Ao,3,3,*Ao_trans);
}

void loop() {
  time2 = millis();
  dt = (time2-time1)*1e-3;
  time1 = time2;  
  t = t + dt;  
  //Serial.println(dt,4);

  Wire.requestFrom(10,24);
  while(Wire.available()) {
// Generating the measured signals.
    xm[0] = Ali_Read();
    xm[1] = Ali_Read();
    xm[2] = Ali_Read();
    xm[3] = Ali_Read();
    xm[4] = Ali_Read(); 

    Dxm = Ali_Read();
  }   
  //Serial.println(xm[3]);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Kalman-Filter
  for(int r=0;r<2;r++){
    zo[r][0] = xm[r] + random(-0.5,0.5);      
  };
  Udo[0][0] = Dxm + 0.1*random(-0.5,0.5);

  Ao[0][2] = cos(xm_hat[2][0]);
  Ao[1][2] = sin(xm_hat[2][0]);

  Ao_trans[2][0] = cos(xm_hat[2][0]);
  Ao_trans[2][1] = sin(xm_hat[2][0]);
   
  MatrixMultiplyAli(*Ao,*Po,3,3,3,*DPo_term1);
  MatrixMultiplyAli(*Po,*Ao_trans,3,3,3,*DPo_term2); 
  MatrixMultiplyAli(*Po,*DPo_term41,3,3,3,*DPo_term42);
  MatrixMultiplyAli(*DPo_term42,*Po,3,3,3,*DPo_term4); 
  
  MatrixMultiplyAli(*Po,*Ko1,3,3,2,*Ko);  

  Dxm_hat_term12[0][0] = cos(xm_hat[2][0])*xm_hat[3][0];
  Dxm_hat_term12[1][0] = sin(xm_hat[2][0])*xm_hat[3][0];
  Dxm_hat_term12[2][0] = Udo[0][0];

  Dxm_hat_term31[0][0] = zo[0][0] - xm_hat[0][0];
  Dxm_hat_term31[1][0] = zo[1][0] - xm_hat[1][0];
  
  MatrixMultiplyAli(*Ko,*Dxm_hat_term31,3,2,1,*Dxm_hat_term3); 
  
  for(int i1=0;i1<3;i1++){
    Dxm_hat[i1][0] = Dxm_hat_term12[i1][0] + Dxm_hat_term3[i1][0];
    for(int j1=0;j1<3;j1++){
      DPo[i1][j1] = DPo_term1[i1][j1] + DPo_term2[i1][j1] + DPo_term3[i1][j1] - DPo_term4[i1][j1];
      Po[i1][j1] = Po[i1][j1] + DPo[i1][j1]*dt;
    }
  }

  xm_hat[0][0] = xm_hat[0][0] + Dxm_hat[0][0]*dt;
  xm_hat[1][0] = xm_hat[1][0] + Dxm_hat[1][0]*dt;
  xm_hat[3][0] = xm_hat[3][0] + Dxm_hat[2][0]*dt;
  xm_hat[2][0] = xm[2];
  xm_hat[4][0] = xm[4];
  
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//Generating the reference signals    
  if (t>10) {
    ydm[0] = 5;
  }
  if (t>70) {
    ydm[0] = 0;
  }
  if (t>40) {
    ydm[1] = 3;
  }
  if (t>100) {
    ydm[1] = 0;
  }
  //ydm[0] = X_Ref;
  //ydm[0] = Y_Ref;  
  
  // Generating the differentiation of reference signal
  for (int j=0;j<2;j++) {
    Diff_err_Ref[j] = omega_Ref[j] - ydm[j];
    if (Diff_err_Ref[j] > 0) {  
      ABS_Diff_err_Ref[j] = Diff_err_Ref[j];                   
    }else  {                              //
      ABS_Diff_err_Ref[j] = -Diff_err_Ref[j];            
    }
    SGN_Diff_err_Ref[j] = Diff_err_Ref[j] / (ABS_Diff_err_Ref[j] + 1e-2);
    tau_Ref[j] = tau_Ref[j] + D_tau_Ref[j]*dt;
    eta_Ref[j] = tau_Ref[j] - (k1*sqrt(ABS_Diff_err_Ref[j])*SGN_Diff_err_Ref[j]);
    omega_Ref[j] = omega_Ref[j] + D_omega_Ref[j]*dt;
    D_tau_Ref[j] = -k2*SGN_Diff_err_Ref[j];
    D_ydm[j] = eta_Ref[j];          
    D_omega_Ref[j] = eta_Ref[j];
  }  
  //Serial.println(D_ydm[1]);  

  err0 = abs(ydm[0] - xm_hat[0][0]) + abs(ydm[1] - xm_hat[1][0]);
  if (err0 > Thresh_T) {
    // Changing the variables.
    if ( (abs(D_ydm[0])+abs(D_ydm[1])) > Thresh ) {
      y_Err = ydm[1] - xm_hat[1][0]; 
      x_Err = ydm[0] - xm_hat[0][0]; 
      XS = y_Err / (x_Err);
      Td = atan(XS);      
      if (abs(Td)>1.50) {
        Td = abs(Td);        
      }
    } 
    //Serial.println(D_ydm[1]);     
    //Serial.println(Td);           
    Pos = (xm_hat[0][0]*cos(xm_hat[2][0])) + (xm_hat[1][0]*sin(xm_hat[2][0]));
    Posd = (ydm[0]*cos(xm_hat[2][0])) + (ydm[1]*sin(xm_hat[2][0]));
    x_T[0] = Pos;
    x_T[1] = xm_hat[3][0]; 
    x_R[0] = xm_hat[2][0]; 
    x_R[1] = xm_hat[4][0]; 
    yd_T[0] = Posd;
    yd_T[1] = u_T[0];
    yd_R[0] = Td;
    yd_R[1] = u_R[0];    
    //Serial.println(xm[1]);
    //Serial.println(yd_T[0]);
    
    // Main loop. generating the control signals. 
    int num = 0; 
    err = yd_R[0] - x_R[0];
    //Serial.println(err);
    if (abs(err) > Thresh_R) {
      
      e_R[num] = yd_R[num] - x_R[num];            
      zeta_R[num] = zeta_R[num] + D_zeta_R[num];
      sigma_R[num] = e_R[num] + zeta_R[num];                                        
      g_R[num] = g_R[num] + D_g_R[num];
      A_R[num] = A_R[num] + D_A_R[num];                                       
      P_R[num] = P_R[num] + D_P_R[num];                                                                   
      u1_R[num] = 0.5*R[num]*B[num]*P_R[num]*sigma_R[num];        
      u2_R[num] = ((1.0f)/B[num])*(D_yd_R[num] - (A_R[num]*x_R[num]) - g_R[num] - zeta_R[num] + (((1.0f)+(((2.0f)/P_R[num])*Q[num])+A_R[num])*sigma_R[num])) - (0.75*R[num]*B[num]*P_R[num]*sigma_R[num]);                    
      u_R[num] = u1_R[num] + u2_R[num];
      D_zeta_R[num] = e_R[num]*dt;
      D_g_R[num] = ( (-Gamma1_R[num]*P_R[num]*sigma_R[num] - rho1*Gamma1_R[num]*g_R[num])*dt );
      D_A_R[num] = ( (-Gamma2_R[num]*P_R[num]*sigma_R[num]*(x_R[num]-sigma_R[num]) - rho2*Gamma2_R[num]*A_R[num])*dt );
      D_P_R[num] = ( (A_R[num]*P_R[num] + P_R[num]*A_R[num] - P_R[num]*B[num]*R[num]*B[num]*P_R[num] + (2.0f)*Q[num])*dt );

      num = num + 1;
      e_R[num] = yd_R[num] - x_R[num];    
      zeta_R[num] = zeta_R[num] + D_zeta_R[num];
      sigma_R[num] = e_R[num] + zeta_R[num];                                        
      g_R[num] = g_R[num] + D_g_R[num];
      A_R[num] = A_R[num] + D_A_R[num];                                       
      P_R[num] = P_R[num] + D_P_R[num];                                                                   
      u1_R[num] = 0.5*R[num]*B[num]*P_R[num]*sigma_R[num];        
      u2_R[num] = ((1.0f)/B[num])*(D_yd_R[num] - (A_R[num]*x_R[num]) - g_R[num] - zeta_R[num] + (((1.0f)+(((2.0f)/P_R[num])*Q[num])+A_R[num])*sigma_R[num])) - (0.75*R[num]*B[num]*P_R[num]*sigma_R[num]);                    
      u_R[num] = u1_R[num] + u2_R[num];
      D_zeta_R[num] = e_R[num]*dt;
      D_g_R[num] = ( (-Gamma1_R[num]*P_R[num]*sigma_R[num] - rho1*Gamma1_R[num]*g_R[num])*dt );
      D_A_R[num] = ( (-Gamma2_R[num]*P_R[num]*sigma_R[num]*(x_R[num]-sigma_R[num]) - rho2*Gamma2_R[num]*A_R[num])*dt );
      D_P_R[num] = ( (A_R[num]*P_R[num] + P_R[num]*A_R[num] - P_R[num]*B[num]*R[num]*B[num]*P_R[num] + (2.0f)*Q[num])*dt );

      // Generating the differentiation of augmented reference signal   
      for (int ii=0;ii<n;ii++) {      
        Diff_err_R[ii] = omega_R[ii] - yd_R[ii];        
        if (Diff_err_R[ii] > 0) {
          ABS_Diff_err_R[ii] = Diff_err_R[ii];      
        }else {
          ABS_Diff_err_R[ii] = -Diff_err_R[ii];
        }
        SGN_Diff_err_R[ii] = Diff_err_R[ii] / (ABS_Diff_err_R[ii] + 1e-2);
        tau_R[ii] = tau_R[ii] + D_tau_R[ii]*dt;
        eta_R[ii] = tau_R[ii] - k1*sqrt(ABS_Diff_err_R[ii])*SGN_Diff_err_R[ii];
        omega_R[ii] = omega_R[ii] + D_omega_R[ii]*dt;
        D_tau_R[ii] = -k2*SGN_Diff_err_R[ii];
        D_yd_R[ii] = eta_R[ii];          
        D_omega_R[ii] = eta_R[ii];
      }       
      u_T[0] = 0;
      u_T[1] = 0;
            
    }else {
      
      e_T[num] = yd_T[num] - x_T[num];    
      //Serial.println(e_T[0]);
      zeta_T[num] = zeta_T[num] + D_zeta_T[num];
      sigma_T[num] = e_T[num] + zeta_T[num];                                        
      g_T[num] = g_T[num] + D_g_T[num];
      A_T[num] = A_T[num] + D_A_T[num];                                       
      P_T[num] = P_T[num] + D_P_T[num];                                                                   
      u1_T[num] = 0.5*R[num]*B[num]*P_T[num]*sigma_T[num];        
      u2_T[num] = ((1.0f)/B[num])*(D_yd_T[num] - (A_T[num]*x_T[num]) - g_T[num] - zeta_T[num] + (((1.0f)+(((2.0f)/P_T[num])*Q[num])+A_T[num])*sigma_T[num])) - (0.75*R[num]*B[num]*P_T[num]*sigma_T[num]);                    
      u_T[num] = u1_T[num] + u2_T[num];
      D_zeta_T[num] = e_T[num]*dt;
      D_g_T[num] = ( (-Gamma1_T[num]*P_T[num]*sigma_T[num] - rho1*Gamma1_T[num]*g_T[num])*dt );
      D_A_T[num] = ( (-Gamma2_T[num]*P_T[num]*sigma_T[num]*(x_T[num]-sigma_T[num]) - rho2*Gamma2_T[num]*A_T[num])*dt );
      D_P_T[num] = ( (A_T[num]*P_T[num] + P_T[num]*A_T[num] - P_T[num]*B[num]*R[num]*B[num]*P_T[num] + (2.0f)*Q[num])*dt );

      num = num + 1;      
      e_T[num] = yd_T[num] - x_T[num];    
      zeta_T[num] = zeta_T[num] + D_zeta_T[num];
      sigma_T[num] = e_T[num] + zeta_T[num];                                        
      g_T[num] = g_T[num] + D_g_T[num];
      A_T[num] = A_T[num] + D_A_T[num];                                       
      P_T[num] = P_T[num] + D_P_T[num];                                                                   
      u1_T[num] = 0.5*R[num]*B[num]*P_T[num]*sigma_T[num];        
      u2_T[num] = ((1.0f)/B[num])*(D_yd_T[num] - (A_T[num]*x_T[num]) - g_T[num] - zeta_T[num] + (((1.0f)+(((2.0f)/P_T[num])*Q[num])+A_T[num])*sigma_T[num])) - (0.75*R[num]*B[num]*P_T[num]*sigma_T[num]);                    
      u_T[num] = u1_T[num] + u2_T[num];
      D_zeta_T[num] = e_T[num]*dt;
      D_g_T[num] = ( (-Gamma1_T[num]*P_T[num]*sigma_T[num] - rho1*Gamma1_T[num]*g_T[num])*dt );
      D_A_T[num] = ( (-Gamma2_T[num]*P_T[num]*sigma_T[num]*(x_T[num]-sigma_T[num]) - rho2*Gamma2_T[num]*A_T[num])*dt );
      D_P_T[num] = ( (A_T[num]*P_T[num] + P_T[num]*A_T[num] - P_T[num]*B[num]*R[num]*B[num]*P_T[num] + (2.0f)*Q[num])*dt );
      
      // Generating the differentiation of augmented reference signal   
      for (int ii=0;ii<n;ii++) {      
        Diff_err_T[ii] = omega_T[ii] - yd_T[ii];       
        if (Diff_err_T[ii] > 0) {
          ABS_Diff_err_T[ii] = Diff_err_T[ii];      
        }else {
          ABS_Diff_err_T[ii] = -Diff_err_T[ii];
        }
        SGN_Diff_err_T[ii] = Diff_err_T[ii] / (ABS_Diff_err_T[ii] + 1e-2);
        tau_T[ii] = tau_T[ii] + D_tau_T[ii]*dt;
        eta_T[ii] = tau_T[ii] - k1*sqrt(ABS_Diff_err_T[ii])*SGN_Diff_err_T[ii];
        omega_T[ii] = omega_T[ii] + D_omega_T[ii]*dt;
        D_tau_T[ii] = -k2*SGN_Diff_err_T[ii];
        D_yd_T[ii] = eta_T[ii];          
        D_omega_T[ii] = eta_T[ii];
      }     
      u_R[0] = 0;
      u_R[1] = 0;
      
    }
        
    U_EM1 = 0.5 * (u_T[1] + u_R[1]);
    U_EM2 = 0.5 * (u_T[1] - u_R[1]);
        
  }else {
    U_EM1 = 0;
    U_EM2 = 0;
  }     
  //Serial.println(U_EM1);  
  
  E1 = U_EM1 * 1000L;
  E2 = U_EM2 * 1000L;
  Ali_I2C_Preparation(E1,E1_Array);
  Ali_I2C_Preparation(E2,E2_Array);
  Wire.beginTransmission(10);
  Ali_Send(E1_Array);
  Ali_Send(E2_Array);
  Wire.endTransmission();

  Serial.print("Time: ");
  Serial.println(t,4);
//  Serial.print("Yd_x: ");
//  Serial.println(ydm[0],4);
//  Serial.print("Yd_y: ");
//  Serial.println(ydm[1],4);
  Serial.print("Err_Pos: ");
  Serial.println(e_T[0],4);
  Serial.print("Err_Rot: ");
  Serial.println(e_R[0],4);
//  Serial.print("g1_Pos: ");
//  Serial.println(g_T[0],4);
//  Serial.print("g2_Pos: ");
//  Serial.println(g_T[1],4);
//  Serial.print("g1_Rot: ");
//  Serial.println(g_R[0],4);
//  Serial.print("g2_Rot: ");
//  Serial.println(g_R[1],4);
//  Serial.print("A1_Pos: ");
//  Serial.println(A_T[0],4);
//  Serial.print("A2_Pos: ");
//  Serial.println(A_T[1],4);
//  Serial.print("A1_Rot: ");
//  Serial.println(A_R[0],4);
//  Serial.print("A2_Rot: ");
//  Serial.println(A_R[1],4);
//  Serial.print("P1_Pos: ");
//  Serial.println(P_T[0],4);
//  Serial.print("P2_Pos: ");
//  Serial.println(P_T[1],4);
//  Serial.print("P1_Rot: ");
//  Serial.println(P_R[0],4);
//  Serial.print("P2_Rot: ");
//  Serial.println(P_R[1],4);
//  Serial.print("U_E1: ");
//  Serial.println(U_EM1,4);
//  Serial.print("U_E2: ");
//  Serial.println(U_EM2,4);
  

}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

float MatrixTransposeAli(float *inp, int n_inp, int m_inp, float *out) {
  for(int i=0;i<n_inp;i++){
    for(int j=0;j<m_inp;j++){
      out[i*m_inp + j] = inp[j*n_inp + i];
    }
  }
}

