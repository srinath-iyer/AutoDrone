#include <Servo.h>
#define BUTTONPIN 13
#define PWM 3
Servo esc;
bool been_pressed = false;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  esc.attach(PWM);
  pinMode(BUTTONPIN, INPUT_PULLUP);
  }

void loop() {
  while(digitalRead(BUTTONPIN)==HIGH && !been_pressed){
    esc.writeMicroseconds(1000);
    Serial.println("High");
  }
  been_pressed = true;
  for(int i = 0; i <= 10; i++){
    esc.writeMicroseconds(1000+100*i);
    delay(250);
  }
  for(int i = 10; i <= 0; i-- ){
    esc.writeMicroseconds(1000+100*i);
    delay(250);
  }

}
