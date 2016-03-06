#include <Servo.h>

/***** Configurables *****/
// Mouth output digital pin
#define MOUTH_UD 46

// Analog sound input
#define AUDIO_INPUT_PIN A0

#define MOUTH_UD_MIN_ANGLE 5
#define MOUTH_UD_MAX_ANGLE 45
/***** End Configurables *****/

Servo mouthUD;
int audioVal;
int outputVal;

void setup()
{
  pinMode(AUDIO_INPUT_PIN, INPUT);
  mouthUD.attach(MOUTH_UD, 600, 2400);
  Serial.begin(9600);
}

void loop()
{
  audioVal = analogRead(AUDIO_INPUT_PIN);
  
  Serial.println(audioVal);
  outputVal = map(audioVal, 0, 1024, MOUTH_UD_MIN_ANGLE, MOUTH_UD_MAX_ANGLE);
  mouthUD.write(outputVal);
  // give us a bit of a delay to servos 
  // to catch up
  delay(150);
}
