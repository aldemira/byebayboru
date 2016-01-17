#include <Servo.h>

int prog_time = 0;
Servo myservo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Servo timings test is starting!");
  myservo.attach(3);
}

void loop() {
  Serial.println("loop starts");
  delay(2000);

  prog_time = millis();
  Serial.println(prog_time);
  Serial.println("Angle 20");
  myservo.write(0);
  prog_time = millis();
  Serial.println(prog_time);
  delay(1000);
  Serial.println("Angle 180");
  myservo.write(180);
  prog_time = millis();
  Serial.println(prog_time-1000);

}
