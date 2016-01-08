#include <Servo.h>

/***** Configurables *****/
// Mouth output digital pin
#define MOUTH_UD 46

// Analog sound input
#define AUDIO_INPUT_PIN 0

#define MOUTH_UD_MIN_ANGLE 0
#define MOUTH_UD_MAX_ANGLE 45
/***** End Configurables *****/

Servo mouthUD;
int audioVal;
int outputVal;

void setup()
{
  mouthUD.attach(MOUTH_UD, 1000, 2000);
}

void loop()
{
  audioVal = analogRead(AUDIO_INPUT_PIN);
  outputVal = map(audioVal, 0, 1023, MOUTH_UD_MIN_ANGLE, MOUTH_UD_MAX_ANGLE);
  mouthUD.write(outputVal);
  // give us a bit of a delay to servos 
  // to catch up
  delay(150);
}
