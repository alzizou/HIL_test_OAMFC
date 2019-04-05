// GPS reading

class ALI_GPS {
  float UTC_time;
  float TTFF;
  float Sat_Num;
  float Course;
    
  int Total_Num_Comma_Cont = 8;
  char str[100];
  char CH1; 
  char CH2; 
  char CH31; 
  char CH32;
  int A00[20][1];
  int A01[20][1];  
  
  String C0 = "AT+IPR=9600";
  String C1 = "AT+CIURC=0";
  String C11 = "AT+CSQ";
  String C2 = "AT+CGNSPWR=1";
  String C3 = "AT+CGNSINF";      
  
  public :
    float Longitude;
    float Latitude;
    float Altitude;
    float GPS_Speed;
    void Init(void);      
    void SetUp(void);   
    void Reading(void);

  private :
    String GPS_Response(void);
};

void ALI_GPS::Init(void) {
  Longitude = 0;
  Latitude = 0;
  Altitude = 0;
  UTC_time = 0;
  TTFF = 0;
  Sat_Num = 0;
  GPS_Speed = 0;
  Course = 0;
  Total_Num_Comma_Cont = 8;            
};

void ALI_GPS::SetUp(void) {    
    Serial1.begin(9600);  
    Serial1.println(C0);
    String str1 = GPS_Response();
    Serial.println(str1);
    delay(500);
    Serial1.println(C1);
    String str2 = GPS_Response();
    Serial.println(str2);
    delay(500);
    Serial1.println(C11);
    String str21 = GPS_Response();
    Serial.println(str21);
    delay(500);
    Serial1.println(C2);
    String str3 = GPS_Response();
    Serial.println(str3);
    delay(500);
}

void ALI_GPS::Reading(void) {
  Serial1.println(C3);
  String str4 = GPS_Response();
  Serial.println(str4);
  delay(2000);
  
//  if (Serial1.available()) {
//    CH1 = Serial1.read();
//    CH2 = Serial1.read();
//    int Num_Comma_Cont = 0;
//    while (Num_Comma_Cont <= (Total_Num_Comma_Cont - 1)) {
//        float GPS_Val = 0;
//        int Comma_Cont = 0;
//        int Dot_Cont = 0;
//        int GPS_Num = 0;
//        while (Comma_Cont == 0) {
//            int GPS_Num0 = 0;
//            while (Dot_Cont == 0) {
//                CH31 = Serial1.read();
//                if (CH31 != '.') {
//                  A00[GPS_Num0][0] = CH31 - '0';
//                  GPS_Num0 = GPS_Num0 + 1;
//                }else {
//                  Dot_Cont = Dot_Cont + 1;
//                }
//            }
//            for (int j=0;j<=GPS_Num0;j++) {
//              GPS_Val = GPS_Val + (A00[j][0] * pow(10,j));
//            }
//      
//            CH32 = Serial1.read();
//            if (CH32 != ',') {
//              A01[GPS_Num][0] = CH32 - '0';
//              GPS_Num = GPS_Num + 1;
//            }
//            else {
//              Comma_Cont = Comma_Cont + 1;
//            }
//        }
//        for (int k=0;k<=GPS_Num;k++) {
//          GPS_Val = GPS_Val + (A01[k][0] / pow(10,k));
//        }
//
//        if (Num_Comma_Cont == 0)
//          Longitude = GPS_Val;
//        else if (Num_Comma_Cont == 1)
//          Latitude = GPS_Val;
//        else if (Num_Comma_Cont == 2)
//          Altitude = GPS_Val;
//        else if (Num_Comma_Cont == 3)
//          UTC_time = GPS_Val;
//        else if (Num_Comma_Cont == 4)
//          TTFF = GPS_Val;
//        else if (Num_Comma_Cont == 5)
//          Sat_Num = GPS_Val;
//        else if (Num_Comma_Cont == 6)
//          GPS_Speed = GPS_Val;
//        else
//          Course = GPS_Val;
//      
//        Num_Comma_Cont = Num_Comma_Cont + 1;
//    }    
//  }
}


String ALI_GPS::GPS_Response(void) {
  String Resp = "";
  while (Serial1.available()) {
    Resp += (char)Serial1.read();
  }
  Resp.trim();
  return Resp;
}
