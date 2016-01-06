#include <Servo.h>

Servo myEyeLid;
int pos = 0;
/* Configurables */
int eyeLidOpenAngle = 0;
int eyeLidCloseAngle = 150;
int eyeLidPin = 9;
/* End Configurables */

int secondBlink = 0;

void setup()
{
  myEyeLid.attach(eyeLidPin, 1000, 2000);
  randomSeed(analogRead(2));
  Serial.begin(9600);
  Serial.println("Eye Blink Test");
  delay(2000);// Give reader a chance to see the output.
}

void loop()
{
  blinkMeEyes();
  delay(3000);
}

void blinkMeEyes()
{
        myEyeLid.write(eyeLidCloseAngle);
        // Anything other than 150ms is too quick
        // for the servos to catch up
        delay(200);
        myEyeLid.write(eyeLidOpenAngle);
        // Make eye blinking a bit more realistic
        // Get a random value and have a 50% chance
        // of blinking the eye again.
        // Kind of makes it less mechanic
        int shallWeBlinkAgain = random(1000,2000);
        Serial.println(shallWeBlinkAgain);
        
        if(shallWeBlinkAgain > 1900 && secondBlink == 1) {
                secondBlink = 0;
                delay(60);
                blinkMeEyes();
        } else {
                // There is a slight chance of shallWeBlinkAgain to be always
                // Smaller than 1500 and it will keep on blinking it's eyes twice.
                secondBlink = 1;
        }
}
