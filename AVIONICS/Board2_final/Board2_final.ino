#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "MPU6050.h"
#include "BMP085.h"

const int SDcard = 10; // SD reader cs pin
const int buzzer_pin = 9;   //작동확인용 부저 9번핀
const long interval = 100;  //100ms 간격으로 기록
unsigned long previousMillis = 0;

File Record;
BMP085 BMP180;    //고도계
MPU6050 AccGyro;  //가속도 및 자이로 센서
TinyGPS Gps;      //GPS 모듈
SoftwareSerial gpsSerial(2, 3); // 추가 시리얼 핀 설정(rx, tx). GPS 모듈과 시리얼 통신 설정

int16_t ax, ay, az; // acceleration
int16_t gx, gy, gz; // gyroscope

float initialPitch = 0.0;
float initialRoll = 0.0;
bool initialAnglesSet = false;
int gps_count=0;   //초기 1회 GPS 안정화를 위한 변수

void setup() {
  Wire.begin();
  if (!SD.begin(SDcard)) {    //SD카드 모듈 정상 작동시 다음 코드로 진행
    while (1);    //SD카드 모듈 정상작동 안할시 루프에서 걸림
  }
  AccGyro.initialize();   //가속도 자이로 센서 초기화
  AccGyro.setI2CBypassEnabled(true);  //뭔지 몰?루
  BMP180.bmp085Calibration();   //고도계 캘리브레이션
  gpsSerial.begin(9600);        //GPS 모듈과 시리얼 통신 시작
  if (!gpsSerial) {      //GPS 모듈 정상 작동시 다음 코드로 진행
    while (1);    //GPS 실패시 루프 걸림
  }

  Record = SD.open("data.csv", FILE_WRITE);   //Sd 카드에 data.csv 파일 기록
  if (Record) {   //데이터 라벨 기록 (Setup 루프에서 최초 1회만)
    Record.print("Time"); Record.print(","); //csv 파일은 콤마와 \n을 기준으로 기록
    Record.print("Fire"); Record.print(","); 
    // BMP180 측정값 저장
    Record.print("Temp"); Record.print(","); 
    Record.print("Press"); Record.print(","); 
    Record.print("Alt"); Record.print(","); 

    // IMU 측정값 저장
    Record.print("ax"); Record.print(","); 
    Record.print("ay"); Record.print(","); 
    Record.print("az"); Record.print(","); 
    Record.print("gx"); Record.print(","); 
    Record.print("gy"); Record.print(","); 
    Record.print("gz"); Record.print(","); 

    // 각도 데이터 저장
    Record.print("Pitch"); Record.print(","); 
    Record.print("Roll"); Record.print(","); 
    Record.print("PitchDiff"); Record.print(",");  
    Record.print("RollDiff"); Record.print(","); 

    // GPS 측정값 저장
    Record.print("Lat"); Record.print(","); 
    Record.print("Lon"); Record.print(","); 
    Record.print("Sat"); Record.print(","); 
    Record.print("Prec"); Record.print(","); 
    Record.print("Year"); Record.print(","); 
    Record.print("Month"); Record.print(","); 
    Record.print("Day"); Record.print(",");
    Record.print("Hour"); Record.print(",");  
    Record.print("Min"); Record.print(","); 
    Record.println("Sec");
  }
  Record.close();     //기록 마무리 
  pinMode(6,INPUT);   //Board1의 10번 핀에서 나오는 사출 신호를 Board2의 6번 핀으로 받음
  analogWrite(buzzer_pin,80); //위의 센서 및 모듈들이 모두 정상작동 시 0.5초 간격으로 부저 3회 작동
  delay(500);
  analogWrite(buzzer_pin,0);
  delay(500);
  analogWrite(buzzer_pin,80);
  delay(500);
  analogWrite(buzzer_pin,0);
  delay(500);
  analogWrite(buzzer_pin,80);
  delay(500);
  analogWrite(buzzer_pin,0);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {   //100ms 간격으로 기록
  // 이전 시간 업데이트
  previousMillis = currentMillis;
  // BMP180 온도, 압력 측정
  float temperature = 0;
  float pressure = 0;
  for (int i = 0; i < 10; i++) {
    temperature += BMP180.bmp085GetTemperature();
    pressure += BMP180.bmp085GetPressure();
    delay(10);
  }
  temperature /= 10;
  pressure /= 10;
  float altitude = BMP180.calcAltitude(pressure);   //Board1과 동일하게 10회 측정값 평균으로 고도 구함

  // MPU6050 데이터 읽기
  AccGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float ax1=ax*100;
  float ay1=ay*100;
  float az1=az*100;     //아래 코드에서 분모가 너무 작아져 에러가 나는 경우 발생해 100을 곱해줌
  // 가속도계 데이터를 사용하여 피치와 롤 계산
  float pitch = atan(ax1*100/ sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI;
  float roll = atan(ay1*100/ sqrt(ax1 * ax1 + az1 * az1)) * 180.0 / PI;

  // 초기 각도 설정
  if (!initialAnglesSet) { 
    initialPitch = pitch;
    initialRoll = roll;
    initialAnglesSet = true;
  }

  // 초기 각도와의 차이 계산
  float pitchDifference = pitch - initialPitch;
  float rollDifference = roll - initialRoll;

  // GPS 데이터 읽기
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  float Lat, Lon, Month, Day, Hour, Minute, Second;
  int Sat, Prec=1000, Year;

  for (unsigned long start = millis(); millis() - start < 50;) {   //50ms
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (Gps.encode(c)) {
        newData = true;
      }
    }
  }

  if (newData) {
    float flat, flon;
    unsigned long age;
    int year;
    byte month, day, hour, minute, second, hundredths;

    Gps.f_get_position(&flat, &flon, &age);
    Lat = (flat == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : flat;
    Lon = (flon == TinyGPS::GPS_INVALID_F_ANGLE) ? 0.0 : flon;
    Sat = (Gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES) ? 0 : Gps.satellites();
    Prec = (Gps.hdop() == TinyGPS::GPS_INVALID_HDOP) ? 0 : Gps.hdop();

    Gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);
    hour += 9; // GMT+9
    if (hour > 24) {
      hour -= 24;
      day += 1;
    }
    Year = year;
    Month = month;
    Day = day;
    Hour = hour;
    Minute = minute;
    Second = second;
  }
  Gps.stats(&chars, &sentences, &failed);

  if(chars!=0 && Prec>0 && Prec<300 && gps_count==0){ //Prec값은 GPS 신호의 안정도 나타냄 300아래로 떨어졌을 때 안정됐다고 판단
    analogWrite(buzzer_pin,80);
    delay(5000);                //GPS 신호 안정화 시 부저 5초 울림 (일반적으로 1~3분 소요)
    analogWrite(buzzer_pin,0);
    gps_count=1;    //안정화 1회 후 더이상 작동하지 않음
  }

  Record = SD.open("data.csv", FILE_WRITE);   //비행 데이터 기록

  if (Record) {
    Record.print(currentMillis); Record.print(","); 
    if(digitalRead(6)==1)  Record.print("Fire");
    else  Record.print(" ");
    Record.print(",");
    // BMP180 측정값 저장
    Record.print(temperature, 2); Record.print(","); 
    Record.print(pressure, 0); Record.print(","); 
    Record.print(altitude, 2); Record.print(","); 

    // IMU 측정값 저장
    Record.print(ax / 16384.0); Record.print(",");  
    Record.print(ay / 16384.0); Record.print(","); 
    Record.print(az / 16384.0); Record.print(","); 
    Record.print(gx / 131.0); Record.print(","); 
    Record.print(gy / 131.0); Record.print(",");  
    Record.print(gz / 131.0); Record.print(","); 

    // 각도 데이터 저장
    Record.print(pitch, 2); Record.print(","); 
    Record.print(roll, 2); Record.print(","); 
    Record.print(pitchDifference, 2); Record.print(","); 
    Record.print(rollDifference, 2); Record.print(","); 

    // GPS 측정값 저장
    Record.print(Lat, 6); Record.print(","); 
    Record.print(Lon, 6); Record.print(","); 
    Record.print(Sat); Record.print(","); 
    Record.print(Prec); Record.print(","); 
    Record.print(Year); Record.print(","); 
    Record.print(Month); Record.print(",");
    Record.print(Day); Record.print(",");
    Record.print(Hour); Record.print(","); 
    Record.print(Minute); Record.print(","); 
    Record.println(Second);

    if (chars == 0) {
      Record.println("** No characters received from GPS: check wiring **");
    }
    Record.close();
  }
  }
}
