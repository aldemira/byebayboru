#include "Thread.h"
#include "ThreadController.h"
#include <Servo.h>


/****** PIN LAYOUT ************
  
  PIN 0  -> Eye Lid R
  PIN 1  -> left eye Up-Down
  PIN 2  -> right eye  Up-Down
  PIN 3  -> left eye  L-R
  PIN 4  -> right eye L-R
  PIN 5  -> Eye Lid L
  PIN 6  -> Mouth R
  PIN 7  -> Mouth L
  PIN 8  -> Head Up-Down
  PIN 9  -> Head L-R
  PIN 10 -> Eyebrow R
  PIN 11 -> Eyebrow L
  PIN 12 -> Ears
  PIN 13 -> LED Heart
  PIN 45 -> Eye brow up
  PIN 46 -> Mouth UP Down
*/
/***** CONFIGURABLE PARAMETERS *****/
/**** PIN definitions as stated above ****/
#define R_EYE_UD 2
#define L_EYE_UD 1
#define L_EYE_LR 3
#define R_EYE_LR 4
#define EYE_LID_L 5
#define EYE_LID_R 0
#define MOUTH_R 6
#define MOUTH_L 7
#define HEAD_UD 8
#define HEAD_LR 9
#define EYE_BROW_R 10
#define EYE_BROW_L 11
#define EARS 12
#define EYE_BROW_UD 45
#define MOUTH_UD 46
#define ledPin 13
/**** End Servo PIN definitions ****/

/**** Audio INPUT PIN definition (Mono) ****/
#define AUDIO_INPUT_PIN A0
/**** End INPUT PIN definition ****/

/**** Angle Definitions ****/
#define LEYE_LID_OPEN_ANGLE 30
#define LEYE_LID_CLOSE_ANGLE 60
#define REYE_LID_OPEN_ANGLE 55
#define REYE_LID_CLOSE_ANGLE 90
#define MOUTH_MIN_ANGLE 5
#define MOUTH_MAX_ANGLE 45
#define INITIAL_EYE_LR_ANGLE 90
/**** End Angle Definitions ****/


/***** End Configurable Stuff *****/

int audioVal;
int outputVal;

// Servo definitions
Servo rEyeLR;
Servo lEyeLR;
Servo eyeLidL;
Servo eyeLidR;
Servo mouthR;
Servo mouthL;
Servo headUD;
Servo headLR;
Servo eyeBrowR;
Servo eyeBrowL;
Servo ears;
Servo eyeBrowU;
Servo mouthUD;
Servo rEyeUD;
Servo lEyeUD;


//Thread definitions

Thread heartbeatTh = Thread();
Thread talkTh = Thread();
Thread buttonTh = Thread();
Thread commonMovesTh = Thread();
ThreadController control1 = ThreadController();


// Boolean for second blink control
int secondBlink = 0;
// Eye blink interval in ms
int blinkinterval = 3000;
// ms to increase the voltage on the heartbeat led
int hearbeatDelay = 50;
// Timer storage
int lastBlinkTime = 0;
int lastEyeMovement = 0;
// Servo pulse widths
int min_pulse = 600;
int max_pulse = 2400;

byte invert = 0;
byte brightness = 0;
int heartbeatDelay = 50;

// Shall we light the heart or dim it
int invertheart = 0;

// Globals to keep the current angle positions
// Used for small eye movements
int curLEyeAngle = INITIAL_EYE_LR_ANGLE;
int curREyeAngle = INITIAL_EYE_LR_ANGLE;

void setup()
{
  //SETUP heart
  pinMode(ledPin, OUTPUT);
  // Make random really random by reading from A2
  randomSeed(analogRead(A2));

  // Thread callback definitions
  heartbeatTh.onRun(heartbeatCallback);
  heartbeatTh.setInterval((hearbeatDelay*10)+10);

  buttonTh.onRun(buttonHandler);
  buttonTh.setInterval((hearbeatDelay*10)+10);

  talkTh.onRun(talkCallback);
  talkTh.setInterval(150);

  commonMovesTh.onRun(commonMovesCallback);
  commonMovesTh.setInterval(3000);

  control1.add(&heartbeatTh);
  control1.add(&buttonTh);
  control1.add(&talkTh);
  control1.add(&commonMovesTh);
    
  //Servo setup
  rEyeUD.attach(R_EYE_UD, min_pulse, max_pulse);
  lEyeUD.attach(L_EYE_UD, min_pulse, max_pulse);
  lEyeLR.attach(L_EYE_LR, min_pulse, max_pulse);
  rEyeLR.attach(R_EYE_LR, min_pulse, max_pulse);
  eyeLidL.attach(EYE_LID_L, min_pulse, max_pulse);
  eyeLidR.attach(EYE_LID_R, min_pulse, max_pulse);
  mouthR.attach(MOUTH_R, min_pulse, max_pulse);
  mouthL.attach(MOUTH_L, min_pulse, max_pulse);
  headUD.attach(HEAD_UD, min_pulse, max_pulse);
  headLR.attach(HEAD_LR, min_pulse, max_pulse);
  eyeBrowR.attach(EYE_BROW_R, min_pulse, max_pulse);
  eyeBrowL.attach(EYE_BROW_L, min_pulse, max_pulse);
  ears.attach(EARS, min_pulse, max_pulse);
  eyeBrowU.attach(EYE_BROW_UD, min_pulse, max_pulse);
  mouthUD.attach(MOUTH_UD, min_pulse, max_pulse);
  // XXX Error checks?

  //eyelids initial position
  eyeLidL.write(LEYE_LID_OPEN_ANGLE);
  eyeLidR.write(REYE_LID_OPEN_ANGLE);
  mouthUD.write(MOUTH_MIN_ANGLE);
}

void loop()
{
  control1.run();
}

void talkCallback()
{
  //talkWithAnalogInput();
}

void buttonHandler()
{
  /*
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
  */
}

void heartbeatCallback()
{
   if(invert == 0) {
    for (int i=0; i<10; i++) {
      analogWrite(ledPin, brightness);
      brightness += 20;
       delay(heartbeatDelay);
    }
    invert = 1;
  } else {
    for(int i=0; i<10; i++) {
      analogWrite(ledPin, brightness);
      brightness -= 20;
      delay(heartbeatDelay);
    }
    invert = 0;
  }  
}

void commonMovesCallback()
{
  int nowTime = millis();
  blinkMeEyes();
  delay(150);
  if(nowTime - lastEyeMovement >= 500 + random(100)) {
    delay(150);
    moveEyesSlightly();
  }

  Serial.println("Blink");
}

// Read audio, and map it to the mouth
// to give a talking impression

void talkWithAnalogInput()
{
  audioVal = analogRead(AUDIO_INPUT_PIN);
  outputVal = map(audioVal, 0, 1023, MOUTH_MIN_ANGLE, MOUTH_MAX_ANGLE);
  mouthUD.write(outputVal);
}

// Eye blink operation
void blinkMeEyes()
{
  eyeLidL.write(LEYE_LID_CLOSE_ANGLE);
  eyeLidR.write(REYE_LID_CLOSE_ANGLE);
	// Anything other than 150ms is too quick 
	// for the servos to catch up
  delay(150);
  eyeLidL.write(LEYE_LID_OPEN_ANGLE);
  eyeLidR.write(REYE_LID_OPEN_ANGLE);
	// Make eye blinking a bit more realistic
	// Get a random value and have a 50% chance 
	// of blinking the eye again. 
	// Kind of makes it less mechanic
	int shallWeBlinkAgain = random(1000,2000);
	if(shallWeBlinkAgain > 1500 && secondBlink == 1) {
		secondBlink = 0;
    eyeBrowU.write(100);
		delay(40);
		blinkMeEyes();
    eyeBrowU.write(70);
    
	} else {
		// There is a slight chance of shallWeBlinkAgain to be always
		// Smaller than 1500 and it will keep on blinking it's eyes twice. 
		secondBlink = 1;
	}
}

// Move eyes slightly to left and right
void moveEyesSlightly()
{
  int curRndNum = random(-20,0);

  if(curLEyeAngle < 20 || curREyeAngle < 20)
	  return;

  curLEyeAngle = curLEyeAngle - curRndNum;
  curREyeAngle = curREyeAngle - curRndNum;
  lEyeLR.write(curLEyeAngle);
  rEyeLR.write(curREyeAngle);

  delay(100);

  curLEyeAngle = curLEyeAngle - abs(curRndNum);
  curREyeAngle = curREyeAngle - abs(curRndNum);
  lEyeLR.write(curLEyeAngle);
  rEyeLR.write(curREyeAngle);
}
