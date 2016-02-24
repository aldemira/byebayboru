#include "Thread.h"
#include "ThreadController.h"
#include <Servo.h>
#include <TimerThree.h>

const int ledPin = 9;// the pin that the LED is attached to
const int led2Pin = 4;
const int servoPin = 2;
const int emptyAnalogPin = 2;
byte brightness = 0;
byte invert = 0;
byte invert2 = 0;
byte brightness2 = 0;
int heartbeatDelay = 50;
int audioVal = 0;
int outputVal = 0;
int buttonstate = 0;

//Thread definitions
Thread heartbeatTh = Thread();
Thread talkTh = Thread();
Thread buttonTh = Thread();
ThreadController control1 = ThreadController();

Servo myservo;

void heartbeatCallback()
{
  if(invert == 0) {
    for (int i=0; i<10; i++) {
      analogWrite(ledPin, brightness);
      brightness += 10;
       delay(heartbeatDelay);
    }
    invert = 1;
  } else {
    for(int i=0; i<10; i++) {
      analogWrite(ledPin, brightness);
      brightness -= 10;
      delay(heartbeatDelay);
    }
    invert = 0;
  }
}

void talkCallback()
{
  audioVal = analogRead(A8);
  Serial.println(audioVal);
  outputVal = map(audioVal, 0, 1023, 0, 110);
  //outputVal = random(0,60);
  //Serial.println(outputVal);
  // Send servo to random positions to simulate 
  // Random mouth movements. 
  myservo.write(outputVal);
  
}

void buttonHandler()
{
  
  buttonstate = digitalRead(12);
  //Serial.println(buttonstate);
  if(buttonstate == HIGH) {
    Serial.println("Button high");
    if(invert2 == 0) {
      for(int i=0;i<10;i++){
        analogWrite(led2Pin, brightness2);
        brightness2 +=10;
        delay(heartbeatDelay);
        invert2 = 1;
      }
    } else {
      for(int i=0;i<10;i++) {
        analogWrite(led2Pin, brightness2);
        brightness2 -=10;
        delay(heartbeatDelay);
        invert2 = 0;
      }
    }
    
  }
}

void timerCallback()
{
  control1.run();
}

void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
  // initialize the ledPin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  myservo.attach(servoPin);
  randomSeed(analogRead(emptyAnalogPin));
  
  heartbeatTh.onRun(heartbeatCallback);
  heartbeatTh.setInterval((heartbeatDelay*10)+10);

  buttonTh.onRun(buttonHandler);
  buttonTh.setInterval((heartbeatDelay*10)+10);
  
  talkTh.onRun(talkCallback);
  talkTh.setInterval(150);

  control1.add(&heartbeatTh);
  control1.add(&buttonTh);
  control1.add(&talkTh);
  
  Timer3.initialize(150000);
  Timer3.attachInterrupt(timerCallback);

  // Button input
  pinMode(12, INPUT);
}

void loop() {
  /*
  if(heartbeatTh.shouldRun())
    heartbeatTh.run();

  if(talkTh.shouldRun())
    talkTh.run();

  if(buttonTh.shouldRun())
    buttonTh.run();
    */
}
