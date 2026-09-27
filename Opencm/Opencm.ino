#include <ArduinoJson.h>
#include <DynamixelWorkbench.h>

#define BAUDRATE  1000000
#define BAUDRATE2  115200

#define DXL_BUS_SERIAL1 "1"            //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 "2"            //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 "3"            //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#define DXL_BUS_SERIAL4 "/dev/ttyUSB0" //Dynamixel on Serial3(USART3)  <-OpenCR

#define DXL_ID18   18
#define DXL_ID1    1
#define DXL_ID2    2
#define DXL_ID3    3
#define DXL_ID4    4
#define DXL_ID5    5
#define DXL_ID6    6
#define DXL_ID7    7
#define DXL_ID8    8
#define DXL_ID9    9
#define DXL_ID10   10
#define DXL_ID11   11

#define DXL_ID12    12
#define DXL_ID13    13
#define DXL_ID14    14
#define DXL_ID15    15
#define DXL_ID16    16
#define DXL_ID17    17

#define vel_ax 400
#define vel_xl 400
#define acc_ax 0
#define acc_xl 0

int motor1 = 512;
int motor2 = 512;
int motor3 = 512;
int motor4 = 512;
int motor5 = 512;
int motor6 = 512;
int motor7 = 512;
int motor8 = 512;
int motor9 = 512;
int motor10 = 512;
int motor11 = 512;
int motor12 = 512;
int motor13 = 512;
int motor14 = 512;
int motor15 = 512;
int motor16 = 512;
int motor17 = 512;
int motor18 = 512;

int d = 5;

StaticJsonDocument<400> doc;

DynamixelWorkbench dxl_wb0, dxl_wb1, dxl_wb2, dxl_wb3, dxl_wb4, dxl_wb5;

void setup() {
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(BAUDRATE2);

  // Initialize the "link" serial port
  // Use a low data rate to reduce the error ratio
  Serial2.begin(BAUDRATE2);

  
  dxl_wb0.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb0.ping(DXL_ID12);
  dxl_wb0.jointMode(DXL_ID12,vel_ax,acc_ax);
  dxl_wb0.ping(DXL_ID18);
  dxl_wb0.jointMode(DXL_ID18,vel_xl,acc_xl);
  dxl_wb0.ping(DXL_ID1);
  dxl_wb0.jointMode(DXL_ID1,vel_xl,acc_xl);

  dxl_wb1.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb1.ping(DXL_ID13);
  dxl_wb1.jointMode(DXL_ID13,vel_ax,acc_ax);
  dxl_wb1.ping(DXL_ID2);
  dxl_wb1.jointMode(DXL_ID2,vel_xl,acc_xl);
  dxl_wb1.ping(DXL_ID3);
  dxl_wb1.jointMode(DXL_ID3,vel_xl,acc_xl);

  dxl_wb2.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb2.ping(DXL_ID14);
  dxl_wb2.jointMode(DXL_ID14,vel_ax,acc_ax);
  dxl_wb2.ping(DXL_ID4);
  dxl_wb2.jointMode(DXL_ID4,vel_xl,acc_xl);
  dxl_wb2.ping(DXL_ID5);
  dxl_wb2.jointMode(DXL_ID5,vel_xl,acc_xl);

  dxl_wb3.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb3.ping(DXL_ID15);
  dxl_wb3.jointMode(DXL_ID15,vel_ax,acc_ax);
  dxl_wb3.ping(DXL_ID6);
  dxl_wb3.jointMode(DXL_ID6,vel_xl,acc_xl);
  dxl_wb3.ping(DXL_ID7);
  dxl_wb3.jointMode(DXL_ID7,vel_xl,acc_xl);

  dxl_wb4.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb4.ping(DXL_ID16);
  dxl_wb4.jointMode(DXL_ID16,vel_ax,acc_ax);
  dxl_wb4.ping(DXL_ID8);
  dxl_wb4.jointMode(DXL_ID8,vel_xl,acc_xl);
  dxl_wb4.ping(DXL_ID9);
  dxl_wb4.jointMode(DXL_ID9,vel_xl,acc_xl);

  dxl_wb5.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb5.ping(DXL_ID17);
  dxl_wb5.jointMode(DXL_ID17,vel_ax,acc_ax);
  dxl_wb5.ping(DXL_ID10);
  dxl_wb5.jointMode(DXL_ID10,vel_xl,acc_xl);
  dxl_wb5.ping(DXL_ID11);
  dxl_wb5.jointMode(DXL_ID11,vel_xl,acc_xl);
}
 
void loop() {

  if ( Serial2.available() )  {
    recibe();
  }

  dxl_wb0.ping(DXL_ID12);
  dxl_wb0.goalPosition(DXL_ID12, motor1);
  delay(d);      
  dxl_wb0.ping(DXL_ID18);
  dxl_wb0.goalPosition(DXL_ID18, motor2);
  delay(d);
  dxl_wb0.ping(DXL_ID1);
  dxl_wb0.goalPosition(DXL_ID1, motor3);
  delay(d);

  dxl_wb1.ping(DXL_ID13);
  dxl_wb1.goalPosition(DXL_ID13, motor4);
  delay(d);      
  dxl_wb1.ping(DXL_ID3);
  dxl_wb1.goalPosition(DXL_ID3, motor5);
  delay(d);
  dxl_wb1.ping(DXL_ID2);
  dxl_wb1.goalPosition(DXL_ID2, motor6);
  delay(d);

  dxl_wb2.ping(DXL_ID14);
  dxl_wb2.goalPosition(DXL_ID14, motor7);
  delay(d);      
  dxl_wb2.ping(DXL_ID5);
  dxl_wb2.goalPosition(DXL_ID5, motor8);
  delay(d);
  dxl_wb2.ping(DXL_ID4);
  dxl_wb2.goalPosition(DXL_ID4, motor9);
  delay(d);

  dxl_wb3.ping(DXL_ID15);
  dxl_wb3.goalPosition(DXL_ID15, motor10);
  delay(d);      
  dxl_wb3.ping(DXL_ID7);
  dxl_wb3.goalPosition(DXL_ID7, motor11);
  delay(d);
  dxl_wb3.ping(DXL_ID6);
  dxl_wb3.goalPosition(DXL_ID6, motor12);
  delay(d);

  dxl_wb4.ping(DXL_ID16);
  dxl_wb4.goalPosition(DXL_ID16, motor13);
  delay(d);      
  dxl_wb4.ping(DXL_ID9);
  dxl_wb4.goalPosition(DXL_ID9, motor14);
  delay(d);
  dxl_wb4.ping(DXL_ID8);
  dxl_wb4.goalPosition(DXL_ID8, motor15);
  delay(d);

  dxl_wb5.ping(DXL_ID17);
  dxl_wb5.goalPosition(DXL_ID17, motor16);
  delay(d);      
  dxl_wb5.ping(DXL_ID11);
  dxl_wb5.goalPosition(DXL_ID11, motor17);
  delay(d);
  dxl_wb5.ping(DXL_ID10);
  dxl_wb5.goalPosition(DXL_ID10, motor18);
  delay(d);
  
}

void recibe() {
  DeserializationError err = deserializeJson(doc, Serial2);
  if (err == DeserializationError::Ok) {
    motor1 = doc["m1"].as<int>();
    motor2 = doc["m2"].as<int>();
    motor3 = doc["m3"].as<int>();
    motor4 = doc["m4"].as<int>();
    motor5 = doc["m5"].as<int>();
    motor6 = doc["m6"].as<int>();
    motor7 = doc["m7"].as<int>();
    motor8 = doc["m8"].as<int>();
    motor9 = doc["m9"].as<int>();
    motor10 = doc["m10"].as<int>();
    motor11 = doc["m11"].as<int>();
    motor12 = doc["m12"].as<int>();
    motor13 = doc["m13"].as<int>();
    motor14 = doc["m14"].as<int>();
    motor15 = doc["m15"].as<int>();
    motor16 = doc["m16"].as<int>();
    motor17 = doc["m17"].as<int>();
    motor18 = doc["m18"].as<int>();
    Serial.print("m1 = ");
    Serial.println(motor1);
    Serial.print("m2 = ");
    Serial.println(motor2);
    Serial.print("m3 = ");
    Serial.println(motor3);
    Serial.print("m4 = ");
    Serial.println(motor4);
    Serial.print("m5 = ");
    Serial.println(motor5);
    Serial.print("m6 = ");
    Serial.println(motor6);
    Serial.print("m7 = ");
    Serial.println(motor7);
    Serial.print("m8 = ");
    Serial.println(motor8);
    Serial.print("m9 = ");
    Serial.println(motor9); 
    Serial.print("m10 = ");
    Serial.println(motor10);
    Serial.print("m11 = ");
    Serial.println(motor11);
    Serial.print("m12 = ");
    Serial.println(motor12); 
    Serial.print("m13 = ");
    Serial.println(motor13);
    Serial.print("m14 = ");
    Serial.println(motor14);
    Serial.print("m15 = ");
    Serial.println(motor15); 
    Serial.print("m16 = ");
    Serial.println(motor16);
    Serial.print("m17 = ");
    Serial.println(motor17);
    Serial.print("m18 = ");
    Serial.println(motor18);
  }
  else 
  {
    Serial.print("deserializeJson() returned ");
    Serial.println(err.c_str());
    while (Serial2.available() > 0)
      Serial2.read();
  }
}
