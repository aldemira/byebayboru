#include <Servo.h>

Servo eyeLid1;
Servo eyeLid2;

int pos = 0;
/* Configurables */
int LeyeLidOpenAngle = 30;
int LeyeLidCloseAngle = 60;
int ReyeLidOpenAngle = 55;
int ReyeLidCloseAngle = 90;
int eyeLidPin1 = 9;
int eyeLidPin2 = 8;
int min_pulse = 600;
int max_pulse = 2400;
/* End Configurables */

int secondBlink = 0;

void setup()
{
  eyeLid1.attach(eyeLidPin1, min_pulse, max_pulse);
  eyeLid2.attach(eyeLidPin2, min_pulse, max_pulse);

  eyeLid1.write(LeyeLidOpenAngle);
  eyeLid2.write(ReyeLidOpenAngle);
  
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
        eyeLid1.write(LeyeLidCloseAngle);
        eyeLid2.write(ReyeLidCloseAngle);
        // Anything other than 150ms is too quick
        // for the servos to catch up
        delay(200);
        eyeLid1.write(LeyeLidOpenAngle);
        eyeLid2.write(ReyeLidOpenAngle);
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
