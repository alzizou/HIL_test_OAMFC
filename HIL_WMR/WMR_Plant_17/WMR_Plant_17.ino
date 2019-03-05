#include <Wire.h>

float U_EM1 = 0;
float U_EM2 = 0;

const float k = 0.1;
const float R = 0.1;
const float L = 0.1;
const float M = 1;
const float J = 0.1;
const float Cv = 0.1;
const float Cw = 0.1;

unsigned long time1 = 0;
unsigned long time2 = 0;
float dt = 0.1;
float t = 0;

float T1 = 0;
float T2 = 0;
float F = 0;
float Tau = 0;
float Dv = 0;
float Dw = 0;
float Dx = 0;
float Dy = 0;
float Dpsi = 0;
float v = 0;
float w = 0;
float x = 0;
float y = 0;
float psi = 0;

int32_t v_Array[4];
int32_t w_Array[4];
int32_t x_Array[4];
int32_t y_Array[4];
int32_t psi_Array[4];
int32_t Dv_Array[4];

float vm;
float wm;
float xm;
float ym;
float psim;
float Dvm;

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
//Serial.println(dt,4);

  x = x + Dx*dt;
  y = y + Dy*dt;
  psi = psi + Dpsi*dt;
  v = v + Dv*dt;
  w = w + Dw*dt;  
}

void event(int h) {      
  while (Wire.available()) {          
    U_EM1 = Ali_Read();    
    U_EM2 = Ali_Read(); 
  } 
  //Serial.println(U_EM1);   
  T1 = k * U_EM1;
  T2 = k * U_EM2;
  F = (1/R) * (T1 + T2);
  Tau = (L/R) * (T1 - T2);    
  Dv = (1/M) * (F - Cv*v + 0.1*sin(0.1*t));
  Dw = (1/J) * (Tau - Cw*w + 0.01*sin(0.1*t));     
  Dx = v * cos(psi);
  Dy = v * sin(psi);
  Dpsi = w;  
}
    
void RequestEvent() {
  vm = v*1000L;
  wm = w*1000L;
  xm = x*1000L;
  ym = y*1000L;
  psim = psi*1000L; 
  Dvm = Dv*1000L;
  Ali_I2C_Preparation(xm,x_Array);
  Ali_I2C_Preparation(ym,y_Array); 
  Ali_I2C_Preparation(psim,psi_Array);
  Ali_I2C_Preparation(vm,v_Array);
  Ali_I2C_Preparation(wm,w_Array);
  Ali_I2C_Preparation(Dvm,Dv_Array);
  Ali_Send(x_Array);
  Ali_Send(y_Array);
  Ali_Send(psi_Array);
  Ali_Send(v_Array);
  Ali_Send(w_Array);
  Ali_Send(Dv_Array);

  Serial.print("X_Pos: ");
  Serial.println(x,4);
  Serial.print("Y_Pos: ");
  Serial.println(y,4);
  Serial.print("Psi: ");
  Serial.println(psi,4);
  Serial.print("U_E1: ");
  Serial.println(U_EM1,4);
  Serial.print("U_E2: ");
  Serial.println(U_EM2,4); 
  
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

