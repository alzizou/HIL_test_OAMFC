#include <Wire.h>

float U_EM1 = 0;
float U_EM2 = 0;
float U_EM3 = 0;
float U_EM4 = 0;

const float Kf = 1e-5;
const float Kt = 0.0435*Kf;
const float L = 0.25;
const float m = 1.95;
const float J[3] = {1.2416e-2,1.2416e-2,2.4832e-2};
const float Kd = 0.01;
const float Ka = 0.01;

unsigned long time1 = 0;
unsigned long time2 = 0;
float dt = 0.1;
float t = 0;

float F1 = 0;
float F2 = 0;
float F3 = 0;
float F4 = 0;

float T1 = 0;
float T2 = 0;
float T3 = 0;
float T4 = 0;

float Dv_x = 0;
float Dv_y = 0;
float Dv_z = 0;

float Dw_phi = 0;
float Dw_theta = 0;
float Dw_psi = 0;

float Dx = 0;
float Dy = 0;
float Dz = 0;

float Dphi = 0;
float Dtheta = 0;
float Dpsi = 0;

float v_x = 0;
float v_y = 0;
float v_z = 0;

float w_phi = 0;
float w_theta = 0;
float w_psi = 0;

float x = 0;
float y = 0;
float z = 0;

float phi = 0;
float theta = 0;
float psi = 0;

float omega[3] = {0,0,0};
float R_omega1_1[3] = {0,0,0};
float R_omega1_2[3] = {0,0,0};
float R_omega1_3[3] = {0,0,0};

float Rq_1[3] = {0,0,0};
float Rq_2[3] = {0,0,0};
float Rq_3[3] = {0,0,0};

float Fx = 0;
float Fy = 0;
float Fz = 0;
float F = 0;
float Vec_F[3] = {0,0,0};
float Vec_Dist[3] = {0,0,0};
float Dist_x = 0;
float Dist_y = 0;
float Dist_z = 0;

float Tau_phi = 0;
float Tau_theta = 0;
float Tau_psi = 0;

float Vec_g[3] = {0,0,9.81};

float EXT_PRD[3] = {0,0,0};
float Jomega[3] = {0,0,0};

int32_t x_Array[4];
int32_t y_Array[4];
int32_t z_Array[4];
int32_t phi_Array[4];
int32_t theta_Array[4];
int32_t psi_Array[4];
int32_t v_x_Array[4];
int32_t v_y_Array[4];
int32_t v_z_Array[4];
int32_t w_phi_Array[4];
int32_t w_theta_Array[4];
int32_t w_psi_Array[4];
int32_t Dv_x_Array[4];
int32_t Dv_y_Array[4];
int32_t Dv_z_Array[4];

float xm;
float ym;
float zm;
float phim;
float thetam;
float psim;
float v_xm;
float v_ym;
float v_zm;
float w_phim;
float w_thetam;
float w_psim;
float Dv_xm;
float Dv_ym;
float Dv_zm;

void setup() {
  Wire.begin(10);
  Wire.setClock(400000L);
  Wire.onReceive(event);
  Wire.onRequest(RequestEvent);

  Serial.begin(115200);

}

void loop() {
  //Generating the sampling time
  time2 = millis();
  dt = (time2-time1)*1e-3;
  time1 = time2;  
  t = t + dt;

  x = x + Dx*dt;
  y = y + Dy*dt;
  z = z + Dz*dt;
  
  phi = phi + Dphi*dt;
  theta = theta + Dtheta*dt;
  psi = psi + Dpsi*dt;   

  v_x = v_x + Dv_x*dt;
  v_y = v_y + Dv_y*dt;
  v_z = v_z + Dv_z*dt;
  
  w_phi = w_phi + Dw_phi*dt;
  w_theta = w_theta + Dw_theta*dt;
  w_psi = w_psi + Dw_psi*dt; 
             
}

void event(int h) {      
  while (Wire.available()) {      
    U_EM1 = Ali_Read();    
    U_EM2 = Ali_Read();   
    U_EM3 = Ali_Read();   
    U_EM4 = Ali_Read();  
  }
  
  F1 = Kf * U_EM1*U_EM1; 
  F2 = Kf * U_EM2*U_EM2;
  F3 = Kf * U_EM3*U_EM3;
  F4 = Kf * U_EM4*U_EM4;
  
  T1 = Kt * U_EM1*U_EM1;
  T2 = Kt * U_EM2*U_EM2;
  T3 = Kt * U_EM3*U_EM3;
  T4 = Kt * U_EM4*U_EM4;    

  omega[0] = w_phi;
  omega[1] = w_theta;
  omega[2] = w_psi;
  
  R_omega1_1[0] = 1;
  R_omega1_1[1] = sin(phi) * tan(theta);
  R_omega1_1[2] = cos(phi) * tan(theta);
  R_omega1_2[0] = 0;
  R_omega1_2[1] = cos(phi);
  R_omega1_2[2] = -sin(phi);
  R_omega1_3[0] = 0;
  R_omega1_3[1] = sin(phi) / (cos(theta) + 1e-3);
  R_omega1_3[2] = cos(phi) / (cos(theta) + 1e-3);        

  Rq_1[0] = cos(psi) * cos(theta);
  Rq_1[1] = (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi));
  Rq_1[2] = (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi));
  Rq_2[0] = sin(psi) * cos(theta);
  Rq_2[1] = (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi));
  Rq_2[2] = (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi));
  Rq_3[0] = -sin(theta);
  Rq_3[1] = cos(theta) * sin(phi);
  Rq_3[2] = cos(theta) * cos(phi); 

  Jomega[0] = J[0] * omega[0];
  Jomega[1] = J[1] * omega[1];
  Jomega[2] = J[2] * omega[2];    
  Ali_External_Product(omega, Jomega, EXT_PRD);

  F = F1 + F2 + F3 + F4;
  Vec_F[0] = 0;
  Vec_F[1] = 0;
  Vec_F[2] = F;
  
  Tau_phi = L * (F2 - F4);    
  Tau_theta = L * (F3 - F1);    
  Tau_psi = (T1 - T2 + T3 - T4);    

  Fx = Ali_Product(Rq_1,Vec_F,3);
  Fy = Ali_Product(Rq_2,Vec_F,3);
  Fz = Ali_Product(Rq_3,Vec_F,3);    

  Vec_Dist[0] = 0;
  Vec_Dist[1] = 0;
  Vec_Dist[2] = 0.1*sin(0.3*t);
   
  Dist_x = Ali_Product(Rq_1,Vec_Dist,3);
  Dist_y = Ali_Product(Rq_2,Vec_Dist,3);
  Dist_z = Ali_Product(Rq_3,Vec_Dist,3);

  Dv_x = (1/m) * (Fx - (Kd*v_x) - m*Vec_g[0] + Dist_x);
  Dv_y = (1/m) * (Fy - (Kd*v_y) - m*Vec_g[1] + Dist_y);
  Dv_z = (1/m) * (Fz - (Kd*v_z) - m*Vec_g[2] + Dist_z);
  
  Dw_phi = (1/J[0])*(Tau_phi - Ka*w_phi - EXT_PRD[0] + 0.1*sin(0.3*t));
  Dw_theta = (1/J[1])*(Tau_theta - Ka*w_theta - EXT_PRD[1] + 0.1*sin(0.3*t));
  Dw_psi = (1/J[2])*(Tau_psi - Ka*w_psi - EXT_PRD[2] + 0.1*sin(0.3*t));  

  Dx = v_x;
  Dy = v_y;
  Dz = v_z; 

  Dphi = Ali_Product(R_omega1_1,omega,3);
  Dtheta = Ali_Product(R_omega1_2,omega,3);
  Dpsi = Ali_Product(R_omega1_3,omega,3);
}


void RequestEvent() {
  xm = x*1000L;
  ym = y*1000L;
  zm = z*1000L;
  phim = phi*1000L;
  thetam = theta*1000L;
  psim = psi*1000L;
  v_xm = v_x*1000L;
  v_ym = v_y*1000L;
  v_zm = v_z*1000L;
  w_phim = w_phi*1000L;
  w_thetam = w_theta*1000L;
  w_psim = w_psi*1000L;
  Dv_xm = Dv_x*1000L;
  Dv_ym = Dv_y*1000L;
  Dv_zm = Dv_z*1000L;
  Ali_I2C_Preparation(xm,x_Array);
  Ali_I2C_Preparation(ym,y_Array);
  Ali_I2C_Preparation(zm,z_Array);
  Ali_I2C_Preparation(phim,phi_Array);
  Ali_I2C_Preparation(thetam,theta_Array);
  Ali_I2C_Preparation(psim,psi_Array);
  Ali_I2C_Preparation(v_xm,v_x_Array);
  Ali_I2C_Preparation(v_ym,v_y_Array);
  Ali_I2C_Preparation(v_zm,v_z_Array);
  Ali_I2C_Preparation(w_phim,w_phi_Array);
  Ali_I2C_Preparation(w_thetam,w_theta_Array);
  Ali_I2C_Preparation(w_psim,w_psi_Array);
  Ali_I2C_Preparation(Dv_xm,Dv_x_Array);
  Ali_I2C_Preparation(Dv_ym,Dv_y_Array);
  Ali_I2C_Preparation(Dv_zm,Dv_z_Array);  
  Ali_Send(x_Array);
  Ali_Send(y_Array);
  Ali_Send(z_Array);
  Ali_Send(phi_Array);
  Ali_Send(theta_Array);
  Ali_Send(psi_Array);
  Ali_Send(v_x_Array);
  Ali_Send(v_y_Array);
  Ali_Send(v_z_Array);
  Ali_Send(w_phi_Array);
  Ali_Send(w_theta_Array);
  Ali_Send(w_psi_Array); 
  Ali_Send(Dv_x_Array);
  Ali_Send(Dv_y_Array);
  Ali_Send(Dv_z_Array);

  Serial.print("X_Pos: ");
  Serial.println(x,4);
  Serial.print("Y_Pos: ");
  Serial.println(y,4);
  Serial.print("Z_Pos: ");
  Serial.println(z,4);
//  Serial.print("Phi: ");
//  Serial.println(phi,4);
//  Serial.print("Theta: ");
//  Serial.println(theta,4);
//  Serial.print("Psi: ");
//  Serial.println(psi,4);
//  Serial.print("V_x: ");
//  Serial.println(v_x,4);
//  Serial.print("V_y: ");
//  Serial.println(v_y,4);
//  Serial.print("V_z: ");
//  Serial.println(v_z,4);
//  Serial.print("W_phi: ");
//  Serial.println(w_phi,4);
//  Serial.print("W_theta: ");
//  Serial.println(w_theta,4);
//  Serial.print("W_psi: ");
//  Serial.println(w_psi,4);
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

float Ali_External_Product(float *inp1, float *inp2, float *outEX) {  

  outEX[0] = (inp1[1] * inp2[2]) - (inp1[2]*inp2[1]);
  outEX[1] = -1 * ((inp1[0] * inp2[2]) - (inp1[2]*inp2[0]));
  outEX[2] = (inp1[0] * inp2[1]) - (inp1[1]*inp2[0]);
}


