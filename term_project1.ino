#include <Pixy2SPI_SS.h>
Pixy2SPI_SS pixy;
#include <Wire.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 TCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//초음파
#define TRIG1 41
#define ECHO1 40
#define TRIG2 43
#define ECHO2 42
#define TRIG3 45
#define ECHO3 44
double distance;
double sn[5] = {0, 0, 0, 0, 0};

//extra value from aron
bool aron_flag = true;
bool aron1_flag = true;
int m1_angle_k = 0;
int m2_angle_k = 0;
int m1_angle_j = 0;
int m2_angle_j = 0;
unsigned long ryu = 0;

// Motor Degree Parameter
int m1_angle = 0;
int m2_angle = 0;
bool m1_sign = 0;
bool m2_sign = 0;

float theta_1 = m1_angle * PI / 180;
float theta_2 = m2_angle * PI / 180;



// Odometry Parameter
float L = 17.2;
float Wheel_R = 3.3;

// Odometry Parameter
float S_R = Wheel_R * theta_1;
float S_L = Wheel_R * theta_2;
float S = (S_R + S_L) / 2;


float theta = (S_R - S_L) / L;
float theta_deg = theta * 180 / PI;

float x = S * cos(3 * theta / 2);
float y = S * sin(3 * theta / 2);


byte U = 0;



int odo_flag = 0;
float S1 = 0;

// Serial Receiver Parameter
bool rcv_status = 0;
bool rcv_ready = 0;
byte rcv_data = 0;
byte rcv_checksum = 0;
byte rx_buffer[7];
byte rx_data[7];
int rcv_count = 0;
int rcv_index = 0;




//모터 uart
byte tx_data[9];
byte checksum = 0;

int8_t i, i_left, i_slope, num, lineSlope[10], lineLeft[10], lineRight[10];
int8_t x0, x1;

int count = 0;
int a = 0;
int b = 0;
unsigned long preMillis = 0;

//블루투스 통신
byte u2_rcv_data = '0';






void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(9600);
  tx_data[0] = 0xFF; //Header 1
  tx_data[1] = 0xFF; //Header 2

  pixy.init();
  pixy.changeProg("line");

  //초음파
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  pinMode(30, OUTPUT);
  digitalWrite(30, LOW);
  if (!TCS.begin()) { // TCS34725 센서가 인식되지 않는 경우
    Serial.println("No TCS34725 found ... check your connections");
    while (1);        // 연결 될 때까지 대기
  }
  else {              // TCS34725 센서가 인식된 경우
    Serial.println("Found sensor");
  }
}



void loop() {


  if (odo_flag == 0) {
    mode1();
    ryu = millis();
  }
  else if (odo_flag == 1)  { // 1번 구역 주차 

    if (aron_flag) {
      memcpy((char*)&m1_angle_k, (char*)rx_data, 2);
      memcpy((char*)&m2_angle_k, (char*)&rx_data[2], 2);
      if (millis() - ryu >= 1500) { //1.5동안 쓰레기값이 검출되므로 1.5초를 쉬고 오도메트리 모드로 넘어간다
        aron_flag = false;
      }
    }
    else odometry1();
  }
  else if (odo_flag == 2)  { //2번 구역 주차 

    if (aron_flag) {
      memcpy((char*)&m1_angle_k, (char*)rx_data, 2);
      memcpy((char*)&m2_angle_k, (char*)&rx_data[2], 2);
      if (millis() - ryu >= 1500) { 
        aron_flag = false;
      }
    }
    else odometry2();
  }
  else if (odo_flag == 3)  { //3번 구역 주차 

    if (aron_flag) {
      memcpy((char*)&m1_angle_k, (char*)rx_data, 2);
      memcpy((char*)&m2_angle_k, (char*)&rx_data[2], 2);
      if (millis() - ryu >= 1500) { 
        aron_flag = false;
      }
    }
    else odometry3();
  }
}


void mode1() {

  Serial.println(count);
  pixy.line.getAllFeatures();
  num = pixy.line.numVectors;

  x0 = pixy.line.vectors[0].m_x0;
  x1 = pixy.line.vectors[0].m_x1;

  if (pixy.line.numBarcodes) { //바코드 0을 인식하는 경우
    if (pixy.line.barcodes->m_code == 0) {
      sendMotor(30, 30);
      //  delay(1500);
      if (millis() - preMillis > 5000) { //바코드가 연속으로 찍혀 count가 계속 증가하는 현상 방지
        count++;
        preMillis = millis();
      }
      if (count == 1)
        a = -10;
      else if (count == 2)
        a = 0;

    }

    else if (pixy.line.barcodes->m_code == 1) { //바코드 1을 인식하는 경우
      sendMotor(30, 30);
      //    delay(1500);
      if (millis() - preMillis > 5000) {
        count++;
        preMillis = millis();
      }
      if (count == 1)
        a = 15;
      else if (count == 2)
        a = 0;
    }
  }
  race();
}



void sendMotor(int m1Spd, int m2Spd) {
  checksum = 0;
  if (m1Spd < 0) {
    tx_data[2] = 0x00; //ccw
    m1Spd *= -1;
  }
  else tx_data[2] = 0x01; //cw
  if (m1Spd > 255) m1Spd = 255;
  tx_data[3] = m1Spd;

  if (m2Spd < 0) {
    tx_data[4] = 0x00; //ccw
    m2Spd *= -1;
  }
  else tx_data[4] = 0x01; //cw
  if (m2Spd > 255) m2Spd = 255;
  tx_data[5] = m2Spd;

  for (int i = 2; i < 6; i++) checksum ^= tx_data[i];
  checksum += 1;
  tx_data[6] = checksum;
  Serial1.write(tx_data, 7);
}

void race() {

  if (Serial3.available()) {
    u2_rcv_data = Serial3.read();
    if (u2_rcv_data == 0x55) {  //U
      uint16_t clear, red, green, blue; // unsigned short int형 변수 선언
      TCS.getRawData(&red, &green, &blue, &clear); // 색상 감지 센서에서 측정 값 받아오기
      int r = map(red, 0, 21504, 0, 1025);    // 빨간색 데이터 값
      int g = map(green, 0, 21504, 0, 1025);  // 초록색 데이터 값
      int b = map(blue, 0, 21504, 0, 1025);   // 파란색 데이터 값

      if ( r > 20) {
        while (true) {
          TCS.getRawData(&red, &green, &blue, &clear); // while문이 진행되는동안 g 값 다시 받기
          int g = map(green, 0, 21504, 0, 1025);
          sendMotor(0, 0);
          odo_flag = 1;
          if (g > 50)   break;
        }
      } else if (g > 50) {
        sendMotor(30, 30);
        delay(150);
        odo_flag = 1;
      }
      if (num) {
        if (x1 - x0 > 0) {
          lineR();
        }
        else if (x1 - x0 < 0) {
          lineL();
        }
      }
      else {
        lineFF();
      }
    }
    if (u2_rcv_data == 0x52) { //R
      uint16_t clear, red, green, blue; // unsigned short int형 변수 선언
      TCS.getRawData(&red, &green, &blue, &clear); // 색상 감지 센서에서 측정 값 받아오기
      int r = map(red, 0, 21504, 0, 1025);    // 빨간색 데이터 값
      int g = map(green, 0, 21504, 0, 1025);  // 초록색 데이터 값
      int b = map(blue, 0, 21504, 0, 1025);   // 파란색 데이터 값

      if ( r > 20) {
        while (true) {
          TCS.getRawData(&red, &green, &blue, &clear); // while문이 진행되는동안 g 값 다시 받기
          int g = map(green, 0, 21504, 0, 1025);
          sendMotor(0, 0);
          odo_flag = 2;
          if (g > 50)   break;
        }
      } else if (g > 50) {
        sendMotor(30, 30);
        delay(150);
        odo_flag = 2;
      }

      if (num) {
        if (x1 - x0 > 0) {
          lineR();
        }
        else if (x1 - x0 < 0) {
          lineL();
        }
      }
      else {
        lineFF();
      }
    }
    if (u2_rcv_data == 0x44) { //D

      uint16_t clear, red, green, blue; // unsigned short int형 변수 선언
      TCS.getRawData(&red, &green, &blue, &clear); // 색상 감지 센서에서 측정 값 받아오기
      int r = map(red, 0, 21504, 0, 1025);    // 빨간색 데이터 값
      int g = map(green, 0, 21504, 0, 1025);  // 초록색 데이터 값
      int b = map(blue, 0, 21504, 0, 1025);   // 파란색 데이터 값

      if ( r > 20) {
        while (true) {
          TCS.getRawData(&red, &green, &blue, &clear); // while문이 진행되는동안 g 값 다시 받기
          int g = map(green, 0, 21504, 0, 1025);
          sendMotor(0, 0);
          odo_flag = 3;
          if (g > 50)   break;
        }
      } else if (g > 50) {
        sendMotor(30, 30);
        delay(150);
        odo_flag = 3;
      }

      if (num) {
        if (x1 - x0 > 0) {
          lineR();
        }
        else if (x1 - x0 < 0) {
          lineL();
        }
      }
      else {
        lineFF();
      }
    }

  }
}


void lineFF() {
  sendMotor(30 + a, 30 + a);
}


void lineR() {
  sendMotor(0, -30 - a);
  // delay(300);
}

void lineL() {
  sendMotor(-35 - a, 0);
  //  delay(500);
}






void serialEvent1() {
  rcv_data = Serial1.read();
  switch (rcv_count) {
    case 0:
      if ((rcv_ready == 0) && (rcv_data == 0xFF)) {
        rcv_count = 1;
      }
      else
        rcv_count = 0;
      break;
    case 1:
      if ((rcv_ready == 0) && (rcv_data == 0xFF)) {
        rcv_count = 2;
        rcv_ready = 1;
      }
      else
        rcv_count = 0;
      break;
    case 2:
      rx_buffer[rcv_index] = rcv_data;
      rcv_index++;
      if (rcv_index > 6) {
        rcv_checksum = 0;
        for (int i = 0; i < 6; i++) {
          rcv_checksum ^= rx_buffer[i];
        }
        rcv_checksum += 1;
        if (rcv_checksum == rx_buffer[rcv_index - 1]) {
          rcv_status = 1;
          memcpy((char*)rx_data, (char*)rx_buffer, 7);
        }
        rcv_count = 0;
        rcv_index = 0;
        rcv_ready = 0;
      }
      break;
    default:
      rcv_count = 0;
      rcv_index = 0;
      rcv_ready = 0;
      break;
  }
}


void turn(int value, float turn_degree) {//속도(속도는 양수만 쳐넣으세용), 각도(거리계산을 이용하며 호의길이를 구해 각도결정) (양수:CCW,음수:CW)

  memcpy((char*)&m1_angle, (char*)rx_data, 2);
  memcpy((char*)&m2_angle, (char*)&rx_data[2], 2);
  float TL = 0;
  float TR = 0;
  float torc = (PI * wheelbase * turn_degree) / 360; //지름이 로봇의 양쪽바퀴사이의 거리인 호의길이
  if (turn_degree <= 0) { //우회전
    if (aron1_flag) {
      memcpy((char*)&m1_angle_j, (char*)rx_data, 2);
      memcpy((char*)&m2_angle_j, (char*)&rx_data[2], 2);
      aron1_flag = 0;

    }

    if ((abs(TL) + abs(TR)) / 2 <= abs(torc)) { //왼쪽바퀴가 돈 거리와 오른쪽바퀴가 돈 거리의 평균 <= 호의길이
      float gap_1 = m1_angle - m1_angle_j; //인코더 값의 차를 계산하여 펄스값과 바퀴원주를 곱해주면 거리나옴
      float gap_2 = m1_angle - m1_angle_j;
      sendMotor (value, -value);
      TR = circumference * (gap_1 / pulseCount);//오른쪽바퀴가 돈 거리
      TL = circumference * (gap_2 / pulseCount);//왼쪽바퀴가 돈거리
    }
    sendMotor(0, 0);
  }

  else if (turn_degree > 0) { //좌회전
    if (aron1_flag) {
      memcpy((char*)&m1_angle_j, (char*)rx_data, 2);
      memcpy((char*)&m2_angle_j, (char*)&rx_data[2], 2);
      aron1_flag = 0;

    }

    if ((abs(TL) + abs(TR)) / 2 <= abs(torc)) { //왼쪽바퀴가 돈 거리와 오른쪽바퀴가 돈 거리의 평균 <= 호의길이
      float gap_1 = m1_angle - m1_angle_j; //인코더 값의 차를 계산하여 펄스값과 바퀴원주를 곱해주면 거리나옴
      float gap_2 = m1_angle - m1_angle_j;
      sendMotor (-value, value);
      TR = circumference * (gap_1 / pulseCount);//오른쪽바퀴가 돈 거리
      TL = circumference * (gap_2 / pulseCount);//왼쪽바퀴가 돈거리
    }
    sendMotor(0, 0);
  }
}


void motor(int value_1, int value_2, int t) { //시간만큼 양쪽모터 속도제어
  preMillis = millis();
  while (millis() - preMillis <= t) {
    sendMotor(value_1, value_2);
  }
}
