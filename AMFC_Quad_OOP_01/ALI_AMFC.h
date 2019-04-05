// Kalman Filter for translational motion and the OAMFC algorithm

class ALI_AMFC {

  static const int n = 12;
  float k1;
  float k2;
  float B[n][1];
  float R[n][1];
  float Q[n][1];
  float P[n][1];
  float A[n][1];
  float g[n][1];
  float x[n][1];    
  float yd[n][1];
  float D_yd[n][1];
  float e[n][1];
  float zeta[n][1];
  float sigma[n][1];
  float u[n][1];
  float Gamma1[n][1];
  float Gamma2[n][1];
  float rho1;
  float rho2;
  
  float P_sigma[n][1];
  
  float U_EM1_1;
  float U_EM1_2;
  float U_EM1_3;
  float U_EM1_4;
  
  float D_zeta[n][1];
  float D_g[n][1];
  float D_A[n][1];
  float D_P[n][1];
  
  float X_Ref;
  float Y_Ref;
  float Z_Ref;
  float ydm[3][1];
  
  float Diff_err[n][1];
  float ABS_Diff_err[n][1];
  float SGN_Diff_err[n][1];
  float omega[n][1];
  float tau[n][1];
  float eta[n][1];
  float D_omega[n][1];
  float D_tau[n][1];
  
  float psi_d;
  float phi_d;
  float theta_d;
  float phi_d0;
  float phi_d1;
  float theta_d0;
  float theta_d1;
  
  float U_Y[4][1];
  
  float Kf;
  float Beta;
  float L;
  float RM1_1[4][1];
  float RM1_2[4][1];
  float RM1_3[4][1];
  float RM1_4[4][1];
  
  int num;
  
  float E1;
  float E2;
  float E3;
  float E4;
  
  int32_t E1_Array[4];
  int32_t E2_Array[4];
  int32_t E3_Array[4];
  int32_t E4_Array[4];
  
  float DPo[6][6];
  float Po[6][6];
  float Qo_val;
  float Ro_val;
  float xm_hat[n][1];
  float Ko[6][3];
  float Ko1[6][3];
  float DPo_term1[6][6];
  float DPo_term2[6][6];
  float DPo_term3[6][6];
  float DPo_term4[6][6];
  float DPo_term41[6][6];
  float DPo_term42[6][6];
  float Dxm_hat[6][1];
  float Dxm_hat_term12[6][1];
  float Dxm_hat_term3[6][1];
  float Dxm_hat_term31[6][1];
  float zo[3][1];
  float Udo[3][1];
  float DPo_time[6][6];
  
  float Rq_1[3][1];
  float Rq_2[3][1];
  float Rq_3[3][1];

  public :
    float U_EM1;
    float U_EM2;
    float U_EM3;
    float U_EM4;
    float xm[n][1];
    float Dxm[3][1];
    float R_Earth_m;
    float dt;
    float t;
    void Kalman_Init(void);
    void AMFC_Init(void);
    void Gen_Measured(float inp_deltat);
    void Kalman_Filter(void);
    void Gen_Refs(void);
    void Main_AMFC(void);
    void Slid_Diff(void);
    
  private :  
    float MatrixMultiplyAli(float *inp1, float *inp2, int n_inp, int m_inp, int p_inp, float*out);
    float Ali_Product(float *inp1, float *inp2, int na);
};


void ALI_AMFC::Kalman_Init(void) {
  R_Earth_m = 6374000;
  Qo_val = 10;
  Ro_val = 0.1;  
  for (int i=0;i<6;i++) {
    xm_hat[i][0] = 0;
    xm_hat[i+6][0] = 0;
    Dxm_hat[i][0] = 0;
    Dxm_hat_term12[i][0] = 0;
    Dxm_hat_term3[i][0] = 0;
    Dxm_hat_term31[i][0] = 0;    
    for (int j=0;j<6;j++) {
      DPo[i][j] = 0;  
      Po[i][j] = 0; 
      DPo_term1[i][j] = 0;   
      DPo_term2[i][j] = 0;
      DPo_term4[i][j] = 0;
      DPo_term42[i][j] = 0;
      DPo_time[i][j] = 0;
      DPo_term3[i][j] = 0;
      DPo_term41[i][j] = 0;
    }
    DPo_term3[i][i] = Qo_val;
    DPo_term41[i][i] = (1.0f)/Ro_val;
    for(int jj=0;jj<3;jj++) {
      Ko[i][jj] = 0;
      Ko1[i][jj] = 0;
      if (i == jj) {
        Ko1[jj][jj] = (1.0f)/Ro_val;
      }      
    }
  } 
  for (int ii=0;ii<3;ii++) {
    zo[ii][0] = 0;
    Udo[ii][0] = 0;
  }  
}


void ALI_AMFC::AMFC_Init(void) {
  dt = 0.1;
  t = 0;  
  num = 0;
  k1 = 1e0;
  k2 = 1e0;
  X_Ref = 0;
  Y_Ref = 0;
  Z_Ref = 0;
  rho1 = 1e-3;
  rho2 = 1e-3; 
  Kf = 1e-5;
  Beta = 1;
  L = 1;
  U_EM1 = 0;
  U_EM2 = 0;
  U_EM3 = 0;
  U_EM4 = 0;
  U_EM1_1 = 0;
  U_EM1_2 = 0;
  U_EM1_3 = 0;
  U_EM1_4 = 0;
  E1 = 0;
  E2 = 0;
  E3 = 0;
  E4 = 0;
  for (int i=0;i<n;i++) {
    B[i][0] = 1;
    R[i][0] = 1;
    Q[i][0] = 1e-2;
    P[i][0] = 1;
    A[i][0] = 0;
    g[i][0] = 0;
    x[i][0] = 0;
    xm[i][0] = 0;
    yd[i][0] = 0;
    D_yd[i][0] = 0;
    e[i][0] = 0;
    zeta[i][0] = 0;
    sigma[i][0] = 0;
    u[i][0] = 0;
    P_sigma[i][0] = 0;
    D_zeta[i][0] = 0;
    D_g[i][0] = 0;
    D_A[i][0] = 0;
    D_P[i][0] = 0;
    Diff_err[i][0] = 0;
    ABS_Diff_err[i][0] = 0;
    SGN_Diff_err[i][0] = 0;
    tau[i][0] = 0;
    eta[i][0] = 0;
    D_omega[i][0] = 0;
    D_tau[i][0] = 0;
    Gamma1[i][0] = 1e-6;
    Gamma2[i][0] = 1e-6;
    omega[i][0] = 0;
    if (i>7) {
      Gamma1[i][0] = 1e+1;
      Gamma2[i][0] = 1e-2;      
    }
  }
  Gamma1[8][0] = 1e-1;
  omega[0][0] = X_Ref;
  omega[1][0] = Y_Ref;
  omega[2][0] = Z_Ref;
  ydm[0][0] = X_Ref;
  ydm[1][0] = Y_Ref;
  ydm[2][0] = Z_Ref;  
  psi_d = 0;
  phi_d = 0;
  theta_d = 0;
  phi_d0 = 0;
  phi_d1 = 0;
  theta_d0 = 0;
  theta_d1 = 0;
  for (int ii=0;ii<4;ii++) {
    U_Y[ii][0] = 0;
  }
  for (int j=0;j<3;j++) {
    Dxm[j][0] = 0;
    for (int jj=0;jj<3;jj++) {
      Rq_1[j][jj] = 0;
    }
  }   
  RM1_1[0][0] = (1/(4*Kf));
  RM1_1[1][0] = 1e-12; 
  RM1_1[2][0] = (-1/(2*L*Kf)); 
  RM1_1[3][0] = (1/(4*Beta*Kf));
  RM1_2[0][0] = (1/(4*Kf));
  RM1_2[1][0] = (1/(2*L*Kf));
  RM1_2[2][0] = 1e-12;
  RM1_2[3][0] = (-1/(4*Beta*Kf));
  RM1_3[0][0] = (1/(4*Kf));
  RM1_3[1][0] = 1e-12;
  RM1_3[2][0] = (1/(2*L*Kf));
  RM1_3[3][0] = (1/(4*Beta*Kf));    
}


void ALI_AMFC::Kalman_Filter(void) {

    float inp_Phi = xm[3][0];
    float inp_Theta = xm[4][0];
    float inp_Psi = xm[5][0];
    Rq_1[0][0] = cos(inp_Psi) * cos(inp_Theta);
    Rq_1[1][0] = (-sin(inp_Psi) * cos(inp_Phi)) + (cos(inp_Psi) * sin(inp_Theta) * sin(inp_Phi));
    Rq_1[2][0] = (sin(inp_Psi) * sin(inp_Phi)) + (cos(inp_Psi) * sin(inp_Theta) * cos(inp_Phi));
    Rq_2[0][0] = sin(inp_Psi) * cos(inp_Theta);
    Rq_2[1][0] = (cos(inp_Psi) * cos(inp_Phi)) + (sin(inp_Psi) * sin(inp_Theta) * sin(inp_Phi));
    Rq_2[2][0] = (-cos(inp_Psi) * sin(inp_Phi)) + (sin(inp_Psi) * sin(inp_Theta) * cos(inp_Phi));
    Rq_3[0][0] = -sin(inp_Theta);
    Rq_3[1][0] = cos(inp_Theta) * sin(inp_Phi);
    Rq_3[2][0] = cos(inp_Theta) * cos(inp_Phi);
    
    for(int r=0;r<3;r++){
      zo[r][0] = xm[r][0];   
      Udo[r][0] = Dxm[r][0];   
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
    Dxm_hat_term12[3][0] = Rq_1[0][0]*Udo[0][0] + Rq_1[1][0]*Udo[1][0] + Rq_1[2][0]*Udo[2][0];  
    Dxm_hat_term12[4][0] = Rq_2[0][0]*Udo[0][0] + Rq_2[1][0]*Udo[1][0] + Rq_2[2][0]*Udo[2][0];  
    Dxm_hat_term12[5][0] = Rq_3[0][0]*Udo[0][0] + Rq_3[1][0]*Udo[1][0] + Rq_3[2][0]*Udo[2][0];  
  
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
    xm_hat[3][0] = xm[3][0];
    xm_hat[4][0] = xm[4][0];
    xm_hat[5][0] = xm[5][0];
    xm_hat[9][0] = xm[9][0];
    xm_hat[10][0] = xm[10][0];
    xm_hat[11][0] = xm[11][0];
}


void ALI_AMFC::Gen_Refs(void) {
    //Generating the reference signals    
    if (t > 20) {
      ydm[0][0] =  2*cos(0.2*(t-20)) - 2; 
      ydm[1][0] = 2*sin(0.2*(t-20));      
    }; 
    ydm[2][0] = 0.2*t;    
  
    phi_d = asin(min(max((-u[7][0]/(U_Y[0][0]+1e-3)),-1),1));      
    theta_d = atan(u[6][0]/(u[8][0] + 1e-3));   
        
    yd[0][0] = ydm[0][0];
    yd[1][0] = ydm[1][0];
    yd[2][0] = ydm[2][0];
    yd[3][0] = phi_d;
    yd[4][0] = theta_d;
    yd[5][0] = psi_d;
    yd[6][0] = u[0][0];
    yd[7][0] = u[1][0];
    yd[8][0] = u[2][0];
    yd[9][0] = u[3][0];
    yd[10][0] = u[4][0];
    yd[11][0] = u[5][0];
}



void ALI_AMFC::Main_AMFC(void) {
    // Main loop. generating the control signals. 
    num = 0; 
    e[num][0] = yd[num][0] - xm_hat[num][0];    
    zeta[num][0] = zeta[num][0] + D_zeta[num][0]*dt;
    sigma[num][0] = e[num][0] + zeta[num][0];                                        
    g[num][0] = g[num][0] + D_g[num][0]*dt;
    A[num][0] = A[num][0] + D_A[num][0]*dt;                                       
    P[num][0] = P[num][0] + D_P[num][0]*dt;    
    P_sigma[num][0] = P[num][0]*sigma[num][0];                                                                 
    u[num][0] = D_yd[num][0] - (A[num][0]*xm_hat[num][0]) - g[num][0] - zeta[num][0] + (((1.0f)+(((2.0f)/P[num][0])*Q[num][0])+A[num][0])*sigma[num][0]) - 0.25*P_sigma[num][0]; 
    D_zeta[num][0] = e[num][0];
    D_g[num][0] = -Gamma1[num][0]*P_sigma[num][0] - rho1*Gamma1[num][0]*g[num][0];
    D_A[num][0] = -Gamma2[num][0]*P_sigma[num][0]*(xm_hat[num][0]-sigma[num][0]) - rho2*Gamma2[num][0]*A[num][0];
    D_P[num][0] = (2.0f)*A[num][0]*P[num][0] - P[num][0]*P[num][0] + (2.0f)*Q[num][0]; 
  
    num = num + 1;
    for(num;num<n;num++) {
      e[num][0] = yd[num][0] - xm_hat[num][0];    
      zeta[num][0] = zeta[num][0] + D_zeta[num][0]*dt;
      sigma[num][0] = e[num][0] + zeta[num][0];                                        
      g[num][0] = g[num][0] + D_g[num][0]*dt;
      A[num][0] = A[num][0] + D_A[num][0]*dt;                                       
      P[num][0] = P[num][0] + D_P[num][0]*dt;   
      P_sigma[num][0] = P[num][0]*sigma[num][0];                                                                   
      u[num][0] = D_yd[num][0] - (A[num][0]*xm_hat[num][0]) - g[num][0] - zeta[num][0] + (((1.0f)+(((2.0f)/P[num][0])*Q[num][0])+A[num][0])*sigma[num][0]) - 0.25*P_sigma[num][0];   
      D_zeta[num][0] = e[num][0];
      D_g[num][0] = -Gamma1[num][0]*P_sigma[num][0] - rho1*Gamma1[num][0]*g[num][0];
      D_A[num][0] = -Gamma2[num][0]*P_sigma[num][0]*(xm_hat[num][0]-sigma[num][0]) - rho2*Gamma2[num][0]*A[num][0];
      D_P[num][0] = (2.0f)*A[num][0]*P[num][0] - P[num][0]*P[num][0] + (2.0f)*Q[num][0];   
    }
    U_Y[0][0] = sqrt(u[6][0]*u[6][0] + u[7][0]*u[7][0] + u[8][0]*u[8][0]);
    U_Y[1][0] = u[9][0];
    U_Y[2][0] = u[10][0];
    U_Y[3][0] = u[11][0];    
    U_EM1 = sqrt(abs(Ali_Product(*RM1_1, *U_Y, 4)));
    U_EM2 = sqrt(abs(Ali_Product(*RM1_2, *U_Y, 4)));
    U_EM3 = sqrt(abs(Ali_Product(*RM1_3, *U_Y, 4)));
    U_EM4 = sqrt(abs(Ali_Product(*RM1_4, *U_Y, 4)));
}


void ALI_AMFC::Slid_Diff(void) {
    // Generating the differentiation of augmented reference signal   
    for (int ii=0;ii<n;ii++) {      
      Diff_err[ii][0] = omega[ii][0] - yd[ii][0];           
      if (Diff_err[ii][0] > 0) {  
        ABS_Diff_err[ii][0] = Diff_err[ii][0];                   
      }else  {                              
        ABS_Diff_err[ii][0] = -Diff_err[ii][0];            
      }
      SGN_Diff_err[ii][0] = Diff_err[ii][0] / (ABS_Diff_err[ii][0] + 1e-2);
      tau[ii][0] = tau[ii][0] + D_tau[ii][0]*dt;
      eta[ii][0] = tau[ii][0] - k1*sqrt(ABS_Diff_err[ii][0])*SGN_Diff_err[ii][0];    
      omega[ii][0] = omega[ii][0] + D_omega[ii][0]*dt;               
      D_tau[ii][0] = -k2*SGN_Diff_err[ii][0];  
      D_yd[ii][0] = eta[ii][0];   
      D_omega[ii][0] = eta[ii][0];  
    }
}


float ALI_AMFC::MatrixMultiplyAli(float *inp1, float *inp2, int n_inp, int m_inp, int p_inp, float*out){
  for(int i=0;i<n_inp;i++){
    for(int j=0;j<p_inp;j++){
      out[i*p_inp + j] = 0;
      for(int k=0;k<m_inp;k++){
        out[i*p_inp + j] = out[i*p_inp + j] + (inp1[i*m_inp + k] * inp2[k*p_inp + j]);
      }
    }
  }  
}


float ALI_AMFC::Ali_Product(float *inp1, float *inp2, int na) {
  float out = 0;
  for (int i=0;(i<na);i++) {
    out = out + (inp1[i] * inp2[i]);    
  }
  return out;
}
