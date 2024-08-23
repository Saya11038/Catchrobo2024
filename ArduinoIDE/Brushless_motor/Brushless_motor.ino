//https://youtu.be/4XYJaPCOJ3Y

#include <Servo.h>

#define ESC_Pin 6     //ESC 出力ピン
char message[50];

//ESC設定(μs)
#define ESC_Min_PWM 1000  //ESC最小PWM(μs)
#define ESC_Max_PWM 2000  //ESC最大PWM(μs)

Servo esc;

void setup() {
  esc.attach(ESC_Pin);
  Serial.begin(9600);

  delay(3000);
  Serial.println("ESC_Max_Setting");
  esc.writeMicroseconds(ESC_Max_PWM);
  delay(2000);
  Serial.println("ESC_Min_Setting");
  esc.writeMicroseconds(ESC_Min_PWM);
  delay(2000);

  pinMode(3, INPUT);
  pinMode(7, OUTPUT);
}

void loop() {

    int vol = 1300;
    esc.writeMicroseconds(vol);
    sprintf(message, "Pulse Width: %d micro sec", vol);
    Serial.println(message);
  

  // if(digitalRead(3) == HIGH) {
  //   int vol = 1300;
  //   esc.writeMicroseconds(vol);
  //   sprintf(message, "Pulse Width: %d micro sec", vol);
  //   Serial.println(message);
  // } else {
  //   int vol = 0;
  //   esc.writeMicroseconds(vol);
  //   sprintf(message, "Pulse Width: %d micro sec", vol);
  //   Serial.println(message);
  // }
}
