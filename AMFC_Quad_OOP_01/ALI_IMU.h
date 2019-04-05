//#include <I2Cdev.h>
#include <HMC5883L.h>
#include <ITG3200.h>
#include <ADXL345.h>
#include <Wire.h> 

class ALI_IMU {

  int Filter_Order;
  float g0;
  //const float deltat = 0.001f;
  
  ADXL345 acc;
  double acc_xyz[3];
  float acc_x; // normalaized
  float acc_y; // normalaized
  float acc_z; // normalaized
  float acc_xg; // m/s^2
  float acc_yg; // m/s^2
  float acc_zg; // m/s^2
  int acc_xN;
  int acc_yN;
  int acc_zN; 
  float acc_x_SUM;
  float acc_y_SUM; 
  float acc_z_SUM;
  float acc_x_AVG; 
  float acc_y_AVG; 
  float acc_z_AVG;
  float acc_x_Filt;
  float acc_y_Filt;
  float acc_z_Filt;   
  float acc_x_HISTORY[5][1];
  float acc_y_HISTORY[5][1];
  float acc_z_HISTORY[5][1];
  float acc_x_Filt_HISTORY[5][1];
  float acc_y_Filt_HISTORY[5][1];
  float acc_z_Filt_HISTORY[5][1];

  ITG3200 gyr;
  float gyr_dx; // deg/sec
  float gyr_dy; // deg/sec
  float gyr_dz; // deg/sec
  float gyr_x; // rad/sec
  float gyr_y; // rad/sec
  float gyr_z; // rad/sec
  int16_t gyr_xN;
  int16_t gyr_yN; 
  int16_t gyr_zN; 
  float gyr_x_SUM; 
  float gyr_y_SUM; 
  float gyr_z_SUM;
  float gyr_x_AVG; 
  float gyr_y_AVG; 
  float gyr_z_AVG;
  float gyr_x_Filt;
  float gyr_y_Filt;
  float gyr_z_Filt;  
  float gyr_x_HISTORY[5][1];
  float gyr_y_HISTORY[5][1];
  float gyr_z_HISTORY[5][1];
  float gyr_x_Filt_HISTORY[5][1];
  float gyr_y_Filt_HISTORY[5][1];
  float gyr_z_Filt_HISTORY[5][1];
  float mx_Filt;
  float my_Filt; 
  float mz_Filt;  
  float mx_HISTORY[5][1];
  float my_HISTORY[5][1];
  float mz_HISTORY[5][1];
  float mx_Filt_HISTORY[5][1];
  float my_Filt_HISTORY[5][1];
  float mz_Filt_HISTORY[5][1];

  HMC5883L mag;
  MagnetometerScaled mag_Data;
  short mag_error;
  int16_t mx;
  int16_t my; 
  int16_t mz;
  float mx_SUM; 
  float my_SUM; 
  float mz_SUM;
  float mx_AVG; 
  float my_AVG; 
  float mz_AVG;

  public:
    int NUM_AVG;
    float acc_x_Filt_OK;
    float acc_y_Filt_OK;
    float acc_z_Filt_OK; 
    float gyr_x_Filt_OK;
    float gyr_y_Filt_OK;
    float gyr_z_Filt_OK;
    float mx_Filt_OK; 
    float my_Filt_OK; 
    float mz_Filt_OK;
    void Init(void);
    void SetUp(void);
    void Reading(void);
    void Averaging(int NUM_inp);   
    void LowPass_Filter(void);
    void Callibrating(void);
    
    
  private: 
    float Main_LowPass_Filter(float inp, float *inp_HISTORY, float *out_HISTORY); 
    
};

void ALI_IMU::Init() {

  NUM_AVG = 1000;
  Filter_Order = 4;
  g0 = 9.81f;

  acc_x_SUM = 0; 
  acc_y_SUM = 0; 
  acc_z_SUM = 0;
  acc_x_AVG = 0; 
  acc_y_AVG = 0; 
  acc_z_AVG = 0;
  
  gyr_x_SUM = 0; 
  gyr_y_SUM = 0; 
  gyr_z_SUM = 0;
  gyr_x_AVG = 0; 
  gyr_y_AVG = 0; 
  gyr_z_AVG = 0;

  mx_SUM = 0; 
  my_SUM = 0; 
  mz_SUM = 0;
  mx_AVG = 0; 
  my_AVG = 0; 
  mz_AVG = 0;

  for (int i=0;i<5;i++) {
    // Acceleration 
    acc_x_HISTORY[i][0] = 0;
    acc_y_HISTORY[i][0] = 0;
    acc_z_HISTORY[i][0] = 0;
    acc_x_Filt_HISTORY[i][0] = 0;
    acc_y_Filt_HISTORY[i][0] = 0;
    acc_z_Filt_HISTORY[i][0] = 0;
    // Gyros
    gyr_x_HISTORY[i][0] = 0;
    gyr_y_HISTORY[i][0] = 0;
    gyr_z_HISTORY[i][0] = 0;
    gyr_x_Filt_HISTORY[i][0] = 0;
    gyr_y_Filt_HISTORY[i][0] = 0;
    gyr_z_Filt_HISTORY[i][0] = 0;
    // Magnetometer
    mx_HISTORY[i][0] = 0;
    my_HISTORY[i][0] = 0;
    mz_HISTORY[i][0] = 0;
    mx_Filt_HISTORY[i][0] = 0;
    my_Filt_HISTORY[i][0] = 0;
    mz_Filt_HISTORY[i][0] = 0; 
  }

};


void ALI_IMU::SetUp(void) {
    Wire.begin();  
    acc.powerOn();
    gyr.init();
    gyr.zeroCalibrate(200,10);
    mag.initCompass();  
    mag_error = mag.setScale(1.3);
    delay(5);
}

void ALI_IMU::Reading(void) {
    acc.readXYZ(&acc_xN, &acc_yN, &acc_zN);     
    acc.getAcceleration(acc_xyz); 
    acc_x = acc_xyz[0];
    acc_y = acc_xyz[1];
    acc_z = acc_xyz[2]; 

    gyr.getXYZ(&gyr_xN,&gyr_yN,&gyr_zN);
    gyr.getAngularVelocity(&gyr_dx,&gyr_dy,&gyr_dz);
    gyr_x = gyr_dx * 3.1416 / 180.0f;
    gyr_y = gyr_dy * 3.1416 / 180.0f;
    gyr_z = gyr_dz * 3.1416 / 180.0f;
    
    mag_Data = mag.readScaledAxis();
    mx = mag_Data.XAxis;
    my = mag_Data.YAxis;
    mz = mag_Data.ZAxis;
}


void ALI_IMU::Averaging(int NUM_inp) {
    acc_x_SUM = acc_x_SUM + acc_x;
    acc_y_SUM = acc_y_SUM + acc_y;
    acc_z_SUM = acc_z_SUM + acc_z;
    acc_x_AVG = acc_x_SUM / (float)NUM_inp;
    acc_y_AVG = acc_y_SUM / (float)NUM_inp;
    acc_z_AVG = acc_z_SUM / (float)NUM_inp;

    gyr_x_SUM = gyr_x_SUM + gyr_x;
    gyr_y_SUM = gyr_y_SUM + gyr_y;
    gyr_z_SUM = gyr_z_SUM + gyr_z;
    gyr_x_AVG = gyr_x_SUM / (float)NUM_inp;
    gyr_y_AVG = gyr_y_SUM / (float)NUM_inp;
    gyr_z_AVG = gyr_z_SUM / (float)NUM_inp; 

    mx_SUM = mx_SUM + mx;
    my_SUM = my_SUM + my;
    mz_SUM = mz_SUM + mz;
    mx_AVG = mx_SUM / (float)NUM_inp;
    my_AVG = my_SUM / (float)NUM_inp;
    mz_AVG = mz_SUM / (float)NUM_inp;
  
}

void ALI_IMU::LowPass_Filter(void) {
    acc_x_Filt = Main_LowPass_Filter(acc_x,*acc_x_HISTORY,*acc_x_Filt_HISTORY);
    acc_y_Filt = Main_LowPass_Filter(acc_y,*acc_y_HISTORY,*acc_y_Filt_HISTORY);
    acc_z_Filt = Main_LowPass_Filter(acc_z,*acc_z_HISTORY,*acc_z_Filt_HISTORY);
    gyr_x_Filt = Main_LowPass_Filter(gyr_x,*gyr_x_HISTORY,*gyr_x_Filt_HISTORY);
    gyr_y_Filt = Main_LowPass_Filter(gyr_y,*gyr_y_HISTORY,*gyr_y_Filt_HISTORY);
    gyr_z_Filt = Main_LowPass_Filter(gyr_z,*gyr_z_HISTORY,*gyr_z_Filt_HISTORY);
    mx_Filt = Main_LowPass_Filter((float)mx,*mx_HISTORY,*mx_Filt_HISTORY);
    my_Filt = Main_LowPass_Filter((float)my,*my_HISTORY,*my_Filt_HISTORY);
    mz_Filt = Main_LowPass_Filter((float)mz,*mz_HISTORY,*mz_Filt_HISTORY);

    // Generating History
    for(int i=(Filter_Order-1);i>0;i--){
      acc_x_HISTORY[i][0] = acc_x_HISTORY[i-1][0];
      acc_y_HISTORY[i][0] = acc_y_HISTORY[i-1][0];
      acc_z_HISTORY[i][0] = acc_z_HISTORY[i-1][0];
      acc_x_Filt_HISTORY[i][0] = acc_x_Filt_HISTORY[i-1][0];
      acc_y_Filt_HISTORY[i][0] = acc_y_Filt_HISTORY[i-1][0];
      acc_z_Filt_HISTORY[i][0] = acc_z_Filt_HISTORY[i-1][0];
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      gyr_x_HISTORY[i][0] = gyr_x_HISTORY[i-1][0];
      gyr_y_HISTORY[i][0] = gyr_y_HISTORY[i-1][0];
      gyr_z_HISTORY[i][0] = gyr_z_HISTORY[i-1][0];
      gyr_x_Filt_HISTORY[i][0] = gyr_x_Filt_HISTORY[i-1][0];
      gyr_y_Filt_HISTORY[i][0] = gyr_y_Filt_HISTORY[i-1][0];
      gyr_z_Filt_HISTORY[i][0] = gyr_z_Filt_HISTORY[i-1][0];
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      mx_HISTORY[i][0] = mx_HISTORY[i-1][0];
      my_HISTORY[i][0] = my_HISTORY[i-1][0];
      mz_HISTORY[i][0] = mz_HISTORY[i-1][0];
      mx_Filt_HISTORY[i][0] = mx_Filt_HISTORY[i-1][0];
      my_Filt_HISTORY[i][0] = my_Filt_HISTORY[i-1][0];
      mz_Filt_HISTORY[i][0] = mz_Filt_HISTORY[i-1][0];
    }
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    acc_x_HISTORY[0][0] = acc_x;
    acc_y_HISTORY[0][0] = acc_y;
    acc_z_HISTORY[0][0] = acc_z;
    acc_x_Filt_HISTORY[0][0] = acc_x_Filt;
    acc_y_Filt_HISTORY[0][0] = acc_y_Filt;
    acc_z_Filt_HISTORY[0][0] = acc_z_Filt;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gyr_x_HISTORY[0][0] = gyr_x;
    gyr_y_HISTORY[0][0] = gyr_y;
    gyr_z_HISTORY[0][0] = gyr_z;
    gyr_x_Filt_HISTORY[0][0] = gyr_x_Filt;
    gyr_y_Filt_HISTORY[0][0] = gyr_y_Filt;
    gyr_z_Filt_HISTORY[0][0] = gyr_z_Filt;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mx_HISTORY[0][0] = mx;
    my_HISTORY[0][0] = my;
    mz_HISTORY[0][0] = mz;
    mx_Filt_HISTORY[0][0] = mx_Filt;
    my_Filt_HISTORY[0][0] = my_Filt;
    mz_Filt_HISTORY[0][0] = mz_Filt;
}


void ALI_IMU::Callibrating(void) {
    acc_x_Filt_OK = acc_x_Filt - acc_x_AVG;
    acc_y_Filt_OK = acc_y_Filt - acc_y_AVG;
    acc_z_Filt_OK = acc_z_Filt - acc_z_AVG;
    acc_xg = acc_x_Filt_OK * g0;
    acc_yg = acc_y_Filt_OK * g0;
    acc_zg = acc_z_Filt_OK * g0;
    //%%%%%%%%%%%%%%%%%%%%%%%%%
    gyr_x_Filt_OK = gyr_x_Filt - gyr_x_AVG;
    gyr_y_Filt_OK = gyr_y_Filt - gyr_y_AVG;
    gyr_z_Filt_OK = gyr_z_Filt - gyr_z_AVG;
    //%%%%%%%%%%%%%%%%%%%%%%%%%%
    mx_Filt_OK = mx_Filt - mx_AVG;
    my_Filt_OK = my_Filt - my_AVG;
    mz_Filt_OK = mz_Filt - mz_AVG;
}


float ALI_IMU::Main_LowPass_Filter(float inp, float *inp_HISTORY, float *out_HISTORY) {
    float out = 0; 
    float out_Num = 0;
    float out_Den = 0; 
    //float Numerator[Filter_Order+1] = {0.0628e-4, 0.1883e-4, 0.1883e-4, 0.0628e-4}; //Order=3
    //float Denominator[Filter_Order+1] = {1, -2.9266, 2.8559, -0.9293}; //Order=3
    float Numerator[Filter_Order+1] = {0.5459e-3, 2.1836e-3, 3.2754e-3, 2.1836e-3, 0.5459e-3}; //Order=4
    float Denominator[Filter_Order+1] = {1, -3.1681, 3.8582, -2.1293, 0.4481}; //Order=4
    //float Numerator[Filter_Order+1] = {0.0351e-0, 0.1755e-0, 0.3509e-0, 0.3509e-0, 0.1755e-0, 0.0351e-0}; //Order=5
    //float Denominator[Filter_Order+1] = {1, -0.5390, 0.8312, -0.2671, 0.1099, -0.0120}; //Order=5
    out_Num = inp * Numerator[0];
    for(int j=0;j<Filter_Order;j++){
      out_Num = out_Num + inp_HISTORY[j] * Numerator[j+1];
      out_Den = out_Den + out_HISTORY[j] * Denominator[j+1];
    }  
    out = out_Num - out_Den;
    return out;
}
