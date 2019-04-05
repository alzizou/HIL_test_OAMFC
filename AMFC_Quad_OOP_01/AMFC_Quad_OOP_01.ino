#include <Servo.h>
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

float EM_Max_Spd = 1000;
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN1 9
#define MOTOR_PIN2 8
#define MOTOR_PIN3 7
#define MOTOR_PIN4 6

#include "ALI_GPS.h"
#include "ALI_IMU.h"
#include "ALI_FILTER.h"
#include "ALI_AMFC.h"

unsigned long time1 = 0;
unsigned long time2 = 0;
float deltat = 0.1;
int NUM = 0;

ALI_GPS GPS_origin;
ALI_GPS GPS;
ALI_IMU IMU;
ALI_FILTER FILTER;
ALI_AMFC AMFC;

void setup() {

  Serial.begin(9600);
  
  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);  
  // Send max output
  delay(2000);
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);
  // Send min output
  delay(2000);
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  
  GPS_origin.Init();  
  GPS.Init();
  GPS_origin.SetUp();
  GPS_origin.Reading();
    
  IMU.Init();
  IMU.SetUp();  
  FILTER.COMP_Init();
  
  AMFC.Kalman_Init();
  AMFC.AMFC_Init();

}

void loop() {
  
  //Generating the sampling time
  time2 = millis();
  deltat = (time2-time1)*1e-3;
  time1 = time2; 
  AMFC.dt = deltat;
  AMFC.t = AMFC.t + AMFC.dt;
  NUM = NUM + 1;

  if (NUM < IMU.NUM_AVG) { 
    
    IMU.Reading();
    IMU.Averaging(NUM);

  } else {

    IMU.Reading();
    IMU.LowPass_Filter();
    IMU.Callibrating();

    // setting the basic values for the complementary filter.
    FILTER.a_x_Madg = IMU.acc_x_Filt_OK;
    FILTER.a_y_Madg = IMU.acc_y_Filt_OK;
    FILTER.a_z_Madg = IMU.acc_z_Filt_OK;
    FILTER.w_x_Madg = IMU.gyr_x_Filt_OK;
    FILTER.w_y_Madg = IMU.gyr_y_Filt_OK;
    FILTER.w_z_Madg = IMU.gyr_z_Filt_OK;
    FILTER.m_x_Madg = IMU.mx_Filt_OK;
    FILTER.m_y_Madg = IMU.my_Filt_OK;
    FILTER.m_z_Madg = IMU.mz_Filt_OK;

    FILTER.Aux_Def();
    FILTER.Normalize();
    FILTER.Jacobian();
    FILTER.Gradient();
    FILTER.Corrected_Gyro(deltat);
    FILTER.Quaternion(deltat);
    FILTER.Mag_Flux();
    FILTER.Euler_Angels();
    
  }
  GPS.Reading();

  // Generating the measured signals.
  AMFC.xm[0][0] = (GPS.Latitude - GPS_origin.Latitude) * (PI/180) * AMFC.R_Earth_m; // x-position from GPS reading 
  AMFC.xm[1][0] = -(GPS.Longitude - GPS_origin.Longitude) * (PI/180) * cos(GPS.Latitude*(PI/180)) * AMFC.R_Earth_m; // y-position from GPS reading 
  AMFC.xm[2][0] = GPS.Altitude - GPS_origin.Altitude;
  AMFC.xm[3][0] = FILTER.Phi; // Phi from IMU reading
  AMFC.xm[4][0] = FILTER.Theta; // Theta from IMU reading
  AMFC.xm[5][0] = FILTER.Psi; // Psi from IMU reading
  AMFC.xm[6][0] = GPS.GPS_Speed; // translational speed from GPS (not used in the algorithm)
  AMFC.xm[7][0] = GPS.GPS_Speed; // translational speed from GPS (not used in the algorithm)
  AMFC.xm[8][0] = GPS.GPS_Speed; // translational speed from GPS (not used in the algorithm)
  AMFC.xm[9][0] = FILTER.w_x_Madg; // rotational speed from IMU reading 
  AMFC.xm[10][0] = FILTER.w_y_Madg; // rotational speed from IMU reading 
  AMFC.xm[11][0] = FILTER.w_z_Madg; // rotational speed from IMU reading 
  AMFC.Dxm[0][0] = FILTER.a_x_Madg; 
  AMFC.Dxm[1][0] = FILTER.a_y_Madg;
  AMFC.Dxm[2][0] = FILTER.a_z_Madg;
  // translational acceleration from IMU reading (supposed that x-axis of the IMU module is aligned with the x direction of the local farme at Quad).

  AMFC.Kalman_Filter();
  AMFC.Gen_Refs();
  AMFC.Main_AMFC();
  AMFC.Slid_Diff();

  // Sending commands to EMs
  float Command_Spd_EM1 = min(abs(AMFC.U_EM1)/EM_Max_Spd,1)*MAX_SIGNAL/(MAX_SIGNAL-MIN_SIGNAL);
  float Command_Spd_EM2 = min(abs(AMFC.U_EM2)/EM_Max_Spd,1)*MAX_SIGNAL/(MAX_SIGNAL-MIN_SIGNAL);
  float Command_Spd_EM3 = min(abs(AMFC.U_EM3)/EM_Max_Spd,1)*MAX_SIGNAL/(MAX_SIGNAL-MIN_SIGNAL);
  float Command_Spd_EM4 = min(abs(AMFC.U_EM4)/EM_Max_Spd,1)*MAX_SIGNAL/(MAX_SIGNAL-MIN_SIGNAL);

  motor1.writeMicroseconds((int)Command_Spd_EM1);     
  motor2.writeMicroseconds((int)Command_Spd_EM2);     
  motor3.writeMicroseconds((int)Command_Spd_EM3);     
  motor4.writeMicroseconds((int)Command_Spd_EM4);

}
