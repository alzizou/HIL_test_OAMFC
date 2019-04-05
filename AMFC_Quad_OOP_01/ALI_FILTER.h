// Low-pass Filter, the Complimentary filter and Euler angles computation

class ALI_FILTER {  
  
  // constants for Euler angles computation
  float norm;
  
  float SEq_1; 
  float SEq_2; 
  float SEq_3; 
  float SEq_4;
  float b_x; 
  float b_z;
  float w_bx; 
  float w_by; 
  float w_bz;
  
  float gyroMeasError;
  float gyroMeasDrift;
  float beta0;
  float zeta0;
  
  float f_1; 
  float f_2; 
  float f_3; 
  float f_4; 
  float f_5; 
  float f_6;
  float J_11or24; 
  float J_12or23; 
  float J_13or22; 
  float J_14or21; 
  float J_32; 
  float J_33; 
  float J_41; 
  float J_42; 
  float J_43; 
  float J_44;
  float J_51; 
  float J_52; 
  float J_53; 
  float J_54; 
  float J_61; 
  float J_62; 
  float J_63; 
  float J_64;
  float SEqHatDot_1; 
  float SEqHatDot_2; 
  float SEqHatDot_3; 
  float SEqHatDot_4;
  float SEqDot_omega_1; 
  float SEqDot_omega_2; 
  float SEqDot_omega_3; 
  float SEqDot_omega_4;
  float w_err_x; 
  float w_err_y; 
  float w_err_z;
  float h_x; 
  float h_y; 
  float h_z;
  
  float halfSEq_1;
  float halfSEq_2; 
  float halfSEq_3; 
  float halfSEq_4;
  float twoSEq_1; 
  float twoSEq_2; 
  float twoSEq_3; 
  float twoSEq_4;
  float twob_x, twob_z;
  float twob_xSEq_1; 
  float twob_xSEq_2; 
  float twob_xSEq_3; 
  float twob_xSEq_4;
  float twob_zSEq_1; 
  float twob_zSEq_2; 
  float twob_zSEq_3; 
  float twob_zSEq_4;
  float SEq_1SEq_2; 
  float SEq_1SEq_3; 
  float SEq_1SEq_4; 
  float SEq_3SEq_4; 
  float SEq_2SEq_3; 
  float SEq_2SEq_4;
  float twom_x;
  float twom_y; 
  float twom_z;

  public :   
    float a_x_Madg; 
    float a_y_Madg; 
    float a_z_Madg;
    float w_x_Madg; 
    float w_y_Madg; 
    float w_z_Madg;
    float m_x_Madg; 
    float m_y_Madg; 
    float m_z_Madg;      
    float Phi; 
    float Theta; 
    float Psi;
    void COMP_Init(void);
    void Aux_Def(void);
    void Normalize(void);
    void Jacobian(void);
    void Gradient(void);
    void Corrected_Gyro(float inp_deltat);
    void Quaternion(float inp_deltat);
    void Mag_Flux(void);
    void Euler_Angels(void);
    
};


void ALI_FILTER::COMP_Init(void) {
  
  norm = 0;
  
  SEq_1 = 1.0f; 
  SEq_2 = 0; 
  SEq_3=0; 
  SEq_4=0;
  b_x = 1.0f; 
  b_z = 0;
  w_bx = 0; 
  w_by = 0; 
  w_bz = 0;
  
  gyroMeasError = 3.1415926535 * (5.0f / 180.0f);
  gyroMeasDrift = 3.1415926535 * (0.2f / 180.0f);
  beta0 = sqrt(3.0f/4.0f) * gyroMeasError;
  zeta0 = sqrt(3.0f/4.0f) * gyroMeasDrift; 

  Phi = 0;
  Theta = 0;
  Psi = 0;
}


void ALI_FILTER::Aux_Def(void) {
    halfSEq_1 = 0.5f * SEq_1;
    halfSEq_2 = 0.5f * SEq_2;
    halfSEq_3 = 0.5f * SEq_3;
    halfSEq_4 = 0.5f * SEq_4;
    twoSEq_1 = 2.0f * SEq_1;
    twoSEq_2 = 2.0f * SEq_2;
    twoSEq_3 = 2.0f * SEq_3;
    twoSEq_4 = 2.0f * SEq_4;
    twob_x = 2.0f * b_x;
    twob_z = 2.0f * b_z;
    twob_xSEq_1 = 2.0f * b_x * SEq_1;
    twob_xSEq_2 = 2.0f * b_x * SEq_2;
    twob_xSEq_3 = 2.0f * b_x * SEq_3;
    twob_xSEq_4 = 2.0f * b_x * SEq_4;
    twob_zSEq_1 = 2.0f * b_z * SEq_1;
    twob_zSEq_2 = 2.0f * b_z * SEq_2;
    twob_zSEq_3 = 2.0f * b_z * SEq_3;
    twob_zSEq_4 = 2.0f * b_z * SEq_4;
    twom_x = 2.0f * m_x_Madg;
    twom_y = 2.0f * m_y_Madg;
    twom_z = 2.0f * m_z_Madg;
}


void ALI_FILTER::Normalize(void) {
    // normalize the accelerometer measurement
    norm = sqrt(a_x_Madg * a_x_Madg + a_y_Madg * a_y_Madg + a_z_Madg * a_z_Madg);
    a_x_Madg /= norm;
    a_y_Madg /= norm;
    a_z_Madg /= norm;
  
    // normalize the magnetometer measurement
    norm = sqrt(m_x_Madg * m_x_Madg + m_y_Madg * m_y_Madg + m_z_Madg * m_z_Madg);
    m_x_Madg /= norm;
    m_y_Madg /= norm;
    m_z_Madg /= norm;  
}


void ALI_FILTER::Jacobian(void) {
    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x_Madg;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y_Madg;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z_Madg;
    f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z *(SEq_2SEq_4 - SEq_1SEq_3) - m_x_Madg;
    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y_Madg;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z_Madg;
    J_11or24 = twoSEq_3;
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
    J_41 = twob_zSEq_3;
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
    J_51 = twob_xSEq_4 - twob_zSEq_2;
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3;
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;
}


void ALI_FILTER::Gradient(void) {
    // Computethe gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
  
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    
}


void ALI_FILTER::Corrected_Gyro(float inp_deltat) {
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
  
    // compute and remove the gyroscope biases
    w_bx += w_err_x * inp_deltat * zeta0;
    w_by += w_err_y * inp_deltat * zeta0;
    w_bz += w_err_z * inp_deltat * zeta0;
    w_x_Madg -= w_bx;
    w_y_Madg -= w_by;
    w_z_Madg -= w_bz;
    
}


void ALI_FILTER::Quaternion(float inp_deltat) {
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x_Madg - halfSEq_3 * w_y_Madg - halfSEq_4 * w_z_Madg;
    SEqDot_omega_2 = halfSEq_1 * w_x_Madg + halfSEq_3 * w_z_Madg - halfSEq_4 * w_y_Madg;
    SEqDot_omega_3 = halfSEq_1 * w_y_Madg - halfSEq_2 * w_z_Madg + halfSEq_4 * w_x_Madg;
    SEqDot_omega_4 = halfSEq_1 * w_z_Madg + halfSEq_2 * w_y_Madg - halfSEq_3 * w_x_Madg;
  
    // compute then integrate the estimated quaternion rate
    SEq_1 += (SEqDot_omega_1 - (beta0 * SEqHatDot_1)) * inp_deltat;
    SEq_2 += (SEqDot_omega_2 - (beta0 * SEqHatDot_2)) * inp_deltat;
    SEq_3 += (SEqDot_omega_3 - (beta0 * SEqHatDot_3)) * inp_deltat;
    SEq_4 += (SEqDot_omega_4 - (beta0 * SEqHatDot_4)) * inp_deltat;
  
    // normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
}


void ALI_FILTER::Mag_Flux(void) {
    // compute magnetic flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2;
    SEq_1SEq_3 = SEq_1 * SEq_3;
    SEq_1SEq_4 = SEq_1 * SEq_4;
    SEq_3SEq_4 = SEq_3 * SEq_4;
    SEq_2SEq_3 = SEq_2 * SEq_3;
    SEq_2SEq_4 = SEq_2 * SEq_4;
    h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 * SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
    
    // normalize the flux vector to have only components in the x and z
    b_x = sqrt(h_x * h_x + h_y * h_y);
    b_z = h_z;
}


void ALI_FILTER::Euler_Angels(void) {
    // Coputation of Euler Angles
    Phi = atan2( (2.0f*SEq_3*SEq_4 - 2.0f*SEq_1*SEq_2),(2.0f*SEq_1*SEq_1 + 2.0f*SEq_4*SEq_4 - 1.0f) );
    Theta = -asin( (2.0f*SEq_2*SEq_4 + 2.0f*SEq_1*SEq_3) );
    Psi = atan2( (2.0f*SEq_2*SEq_3 - 2.0f*SEq_1*SEq_4),(2.0f*SEq_1*SEq_1 + 2.0f*SEq_2*SEq_2 - 1.0f) );
}
