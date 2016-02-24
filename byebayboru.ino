#include <Servo.h>

/****** PIN LAYOUT ************
  
  PIN 0  -> Unused
  PIN 1  -> right eye Up-Down
  PIN 2  -> left eye  Up-Down
  PIN 3  -> left eye  L-R
  PIN 4  -> right eye L-R
  PIN 5  -> Eye Lids (L+R)
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
#define R_EYE_UD 1
#define L_EYE_UD 2
#define L_EYE_LR 3
#define R_EYE_LR 4
#define EYE_LIDS 5
#define MOUTH_R 6
#define MOUTH_L 7
#define HEAD_UD 8
#define HEAD_LR 9
#define EYE_BROW_R 10
#define EYE_BROW_L 11
#define EARS 12
#define EYE_BROW_UP 45
#define MOUTH_UD 46
/**** End Servo PIN definitions ****/

/**** Audio INPUT PIN definition (Mono) ****/
#define AUDIO_INPUT_PIN 0
/**** End INPUT PIN definition ****/

/**** Angle Definitions ****/
#define EYE_LID_OPEN_ANGLE 20
#define EYE_LID_CLOSE_ANGLE 70
#define MOUTH_MIN_ANGLE 0
#define MOUTH_MAX_ANGLE 45
#define INITIAL_EYE_LR_ANGLE 90
/**** End Angle Definitions ****/


/***** End Configurable Stuff *****/

int audioVal;
int outputVal;

// Servo definitions
Servo rEyeLR;
Servo eyeLids;
Servo mouthR;
Servo mouthL;
Servo headUD;
Servo headLR;
Servo eyeBrowR;
Servo eyeBrowL;
Servo ears;
Servo eyeBrowU;
Servo mouthUD;

// Boolean for second blink control
int secondBlink = 0;

// Timer storage
int lastBlinkTime = 0;
int lastEyeMovement = 0;

// Globals to keep the current angle positions
// Used for small eye movements
int curLEyeAngle = INITIAL_EYE_LR_ANGLE;
int curREyeAngle = INITIAL_EYE_LR_ANGLE;

void setup()
{
  // Make random really random by reading from A2
  randomSeed(analogRead(2));

  rEyeUD.attach(R_EYE_UD, 1000, 2000);
  lEyeUD.attach(L_EYE_UD, 1000, 2000);
  lEyeLR.attach(L_EYE_LR, 1000, 2000);
  rEyeLR.attach(R_EYE_LR, 1000, 2000);
  eyeLids.attach(EYE_LIDS, 1000, 2000);
  mouthR.attach(MOUTH_R, 1000, 2000);
  mouthL.attach(MOUTH_L, 1000, 2000);
  headUD.attach(HEAD_UD, 1000, 2000);
  headLR.attach(HEAD_LR, 1000, 2000);
  eyeBrowR.attach(EYE_BROW_R, 1000, 2000);
  eyeBrowL.attach(EYE_BROW_L, 1000, 2000);
  ears.attach(EARS, 1000, 2000);
  eyeBrowU.attach(EYE_BROW_UD, 1000, 2000);
  mouthUD.attach(MOUTH_UD, 1000, 2000);
  // XXX Error checks?

 //eyelids initial angle
 // XXX do we need to define initial angle for each servo?
 eyelids.write(eyeLidOpenAngle);
}

void loop()
{

  talkWithAnalogInput();

  // XXX What will be the assumed initial servo angles?
  nowTime = millis();
  if(nowTime - lastBlinkTime >= 5000)
        blinkMeEyes();

  if(nowTime - lastEyeMovement >= 500 + random(100))
        moveEyesSlightly();

  // Wait 150ms between each analog sample
  delay(150);
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
        eyelids.write(eyeLidCloseAngle);
	// Anything other than 150ms is too quick 
	// for the servos to catch up
        delay(150);
        eyelids.write(eyeLidOpenAngle);
	// Make eye blinking a bit more realistic
	// Get a random value and have a 50% chance 
	// of blinking the eye again. 
	// Kind of makes it less mechanic
	int shallWeBlinkAgain = random(1000,2000);
	if(shallWeBlinkAgain > 1500 && secondBlink == 1) {
		secondBlink = 0;
		delay(40);
		blinkMeEyes();
	} else {
		// There is a slight chance of shallWeBlinkAgain to be always
		// Smaller than 1500 and it will keep on blinking it's eyes twice. 
		secondBlink = 1;
	}
}

// Move eyes slightly to left and right
void moveEyesSlightly()
{
  int curRndNum = random(-5,0);

  if(curLEyeAngle < 5 || curREyeAngle < 5)
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
