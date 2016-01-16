#include <Servo.h>

/* configurables */
int min_pulse = 1000;
int max_pulse = 2000;
int pinno = 3;
/* end configurables */

Servo myservo;
void setup() {
  myservo.attach(pinno, min_pulse, max_pulse);
  myservo.write(0);
}

void loop() {
}
