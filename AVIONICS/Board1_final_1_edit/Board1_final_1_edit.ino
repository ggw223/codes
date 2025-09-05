#include<SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "MPU6050.h"  // scl-A5; sda-A4;
#include "BMP085.h"

SoftwareSerial HC12(2,3); //rx tx, 강제사출용 통신모듈
BMP085 BMP180;


float initalt;    //초기 고도 저장
float maxalt;     //최고 고도 저장
float safealt;    //안전 고도 저장 ---> 새로 추가
int trigger_alt=15;    //최고 고도 기준 6m 하강 시 사출-->15m로 바꿈
char fire = 'N';    //낙하산 깅제사출 여부
bool is_safealt_over = false;
bool cdn1 = false;
bool cdn2 = false;
bool cdn3 = false;

unsigned long previousMillis = 0; // 마지막 업데이트 시간
const long interval = 100; // 100ms 간격으로 데이터 수집

int relay_pin=12;     //낙하산 사출 릴레이 12번에 연결
int buzzer_pin=9;     //Board1 상태 확인용 부저 9번에 연결

void setup(){
  pinMode(10,OUTPUT);   //10번 핀은 Board2와 연결되어 사출 명령시 HIGH신호 전달
  digitalWrite(10,LOW);
  HC12.begin(19200);    //강제사출용 통신모듈 통신속도
  Wire.begin();         //BMP085(고도센서)와 I2C 통신 시작
  pinMode(relay_pin,OUTPUT);
  pinMode(buzzer_pin,OUTPUT);
  BMP180.bmp085Calibration();   //BMP085 캘리브레이션
  delay(10);
  float inittemp = 0;
  float initpress = 0;      //초기 온도 및 압력 읽어오기
  for (int i = 0; i < 10; i++) {
    inittemp += BMP180.bmp085GetTemperature();  //온도를 안 읽어오면 고도가 튀는 현상 발생
    initpress += BMP180.bmp085GetPressure();
    delay(10);
  }
  inittemp = inittemp / 10; 
  initpress = initpress / 10;     //초기 온도 및 압력 10회 측정 평균치로 저장
  initalt = BMP180.calcAltitude(initpress);   //초기 온도 및 압력 기반 초기 고도 계산
  safealt = initalt + 150;   //안전 고도는 initalt + 150으로 설정 ---> 새로 추가한 내용
  maxalt=initalt;     //최고고도에 초기 고도값 저장
  analogWrite(buzzer_pin,80);
  delay(5000);                //부저 5초 작동으로 Board1 전원 켜짐 확인
  analogWrite(buzzer_pin,0);
  delay(3000);
  if(initalt<50){             //3초 대기 후 초기 고도값에 따라 0.5초 간격으로 부저 작동
    analogWrite(buzzer_pin,80);   //고도 50m 미만인 경우 1회
    delay(500);
    analogWrite(buzzer_pin,0);

  }
  else if(initalt<100){         //고도 100m 미만인 경우 2회
    analogWrite(buzzer_pin,80);
    delay(500);
    analogWrite(buzzer_pin,0);
    delay(500);
    analogWrite(buzzer_pin,80);
    delay(500);
    analogWrite(buzzer_pin,0);

  }
  else{                       //고도 100m 이상인 경우 3회
    analogWrite(buzzer_pin,80);
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

}
void relay();

float calcMedian(float* values) { //3개짜리 중앙값
  float a = values[0];
  float b = values[1];
  float c = values[2];
  if ((a >= b && a <= c) || (a <= b && a >= c)) { return a; }
  else if ((b >= a && b <= c) || (b <= a && b >= c)) { return b; }
  else { return c; }
}

float calcAvg(float* medians) { //중앙값으로 나온 3개(123 234 345) -> avg1 , (234, 345, 456) -> avg2... avg5
  return ((medians[0] + medians[1] + medians[2]) / 3);
}
bool checkDescend(float* reps) {
  for (int i = 1; i < 5; ++i) {
    if (reps[i - 1] < reps[i]) { return false; }
  } return true;
}
void updateArr(float* arr, int size, float newalt) {
  for (int i = 0; i < size - 1; ++i) {
    arr[i] = arr[i + 1];
  }
  arr[size - 1] = newalt;
}
uint8_t turn_count = 0; //두칸 건너뛰기용, 아두이노에선 바로 사용가능한 자료형임, cpp에선 stdint.h필요
float alt_vals[3] = { initalt };

float medians[3] = { initalt };

float avgs[5] = { initalt };

uint8_t CDN3_count = 0;

void loop(){
  if(HC12.available()){         //강제사출 신호 최우선으로 읽어옴
    fire=(char)HC12.read();     //통신모듈 수신값을 사출 여부 변수에 저장
  }                             //강제 사출이 필요한 경우 송신 모듈에서 'Y' 전송
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {   //100ms 간격으로 고도 측정 수행
    previousMillis = currentMillis;

    float temperature = 0;
    float pressure = 0;
    for (int i = 0; i < 10; i++) {
      temperature += BMP180.bmp085GetTemperature();
      pressure += BMP180.bmp085GetPressure();
      delay(2);
    }
    temperature = temperature / 10;
    pressure = pressure / 10;             //2ms 간격으로 10회 평균치 측정
    float altitude = BMP180.calcAltitude(pressure); //현재 고도 저장
    

    if(altitude>maxalt){      //현재고도가 최고고도보다 높으면 최고고도 갱신
      maxalt=altitude;
    }



    //사출판정부
    if(altitude > safealt)
      is_safealt_over = true;
     

    if(is_safealt_over){
      if (altitude < maxalt-trigger_alt)    //현재고도가 최고고도보다 6m 이상 낮으면 --> 15m로 바꿈
        {cdn1 = true; }                        //조건 1 만족
     
      updateArr(alt_vals, 3, altitude);
      updateArr(medians, 3, calcMedian(alt_vals));
      turn_count = (turn_count + 1) % 2; //두칸 건너뛴다는건 매 격루프에만 갱신
      if (turn_count == 0) { //격루프일때의 rep의 감소여부따지기
        updateArr(avgs, 5, calcAvg(medians)); //평균5개 -> 하나의 대표 묶음, 두번에 한번 갱신이므로 1 3 5 7 9번째  평균
        if (checkDescend(avgs)) 
        { cdn2 = true;}  // 조건2 만족

      if (altitude > maxalt) {
        CDN3_count = 0;   
      }
      else { CDN3_count = (CDN3_count + 1) % 30; }
      if (CDN3_count == 29) {
        cdn3 = true;
      }
      
      
    }
     //현재고도가 안전고도보다 높거나 같을 때부터 낙하산 사출 가능 --> 새로 추가
    if(is_safealt_over && ((cdn1&&cdn2)||(cdn2&&cdn3)||(cdn3&&cdn1))){
      fire = 'Y';
    }

   
    if (fire=='Y'){           //낙하산 사출 명령이 내려졌다면
      digitalWrite(10,HIGH);  //화약 점화를 위한 릴레이 작동
      relay();
    }
  
  }
}
}
void relay(){       //3초간 화약뭉치에 전류 흘림
  digitalWrite(relay_pin,HIGH);
  delay(3000);
  digitalWrite(relay_pin,LOW);
}
