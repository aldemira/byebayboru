#include <Servo.h>
const int ledPin = 9;      // the pin that the LED is attached to
const int servoPin = 2;
const int emptyAnalogPin = 2;
byte brightness = 0;


Servo myservo;
void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
  // initialize the ledPin as an output:
  pinMode(ledPin, OUTPUT);
  myservo.attach(servoPin);
  randomSeed(analogRead(emptyAnalogPin));
}

void loop() {
  int servoAngle = random(0,180);
  Serial.println(servoAngle);
  // Send servo to random positions to simulate 
  // Random mouth movements. 
  myservo.write(servoAngle);

  for (int i=0; i<10; i++) {
     analogWrite(ledPin, brightness);
     brightness += 10;
     delay(50);
  }
  servoAngle = random(0,180);
  Serial.println(servoAngle);
  // Send servo to random positions to simulate 
  // Random mouth movements. 
  myservo.write(servoAngle);
  //delay(100);
  for(int i=0; i<10; i++) {
    analogWrite(ledPin, brightness);
    brightness -= 10;
    delay(50);
  }
  servoAngle = random(0,180);
  Serial.println(servoAngle);
  // Send servo to random positions to simulate 
  // Random mouth movements. 
  myservo.write(servoAngle);
 
}
