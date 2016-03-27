/*
    Bye Bay Boru is an effort to create a robotic face
    Copyright (C) 2016 Aldemir Akpinar
    Copyright (C) 2016 Yüz Yüze Sanat (https://www.facebook.com/yuzyuzesanat)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "Thread.h"
#include "ThreadController.h"
#include <Servo.h>
#include <Wire.h>


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
#define EYE_LID_L 0
#define EYE_LID_R 5
#define MOUTH_R 6
#define MOUTH_L 7
#define HEAD_UD 8
#define HEAD_LR 9
#define EYE_BROW_R 10
#define EYE_BROW_L 11
#define EARS 12
#define EYE_BROW_UD 45
#define MOUTH_UD 46
#define HEART_LED 13
/**** End Servo PIN definitions ****/

/**** Audio INPUT PIN definition (Mono) ****/
#define AUDIO_INPUT_PIN A0
/**** Accelerometer INPUTS ****/
#define ACC_X_INPUT_PIN A2
#define ACC_Y_INPUT_PIN A3
#define ACC_Z_INPUT_PIN A4
/***** Button Inputs *******/
#define BUTTON1_INPUT_PIN A5
#define BUTTON2_INPUT_PIN A6
/****** Button Inputs ******/
/**** End INPUT PIN definition ****/
/**** Servo Angle Definitions ****/
#define LEYE_LID_OPEN_ANGLE 60
#define LEYE_LID_CLOSE_ANGLE 85
#define REYE_LID_OPEN_ANGLE 40
#define REYE_LID_CLOSE_ANGLE 60
#define MOUTH_MIN_ANGLE 5
#define MOUTH_MAX_ANGLE 25
#define INITIAL_EYE_LR_ANGLE 90
#define EARS_INITIAL_ANGLE 90
#define EARS_FINAL_ANGLE 120
#define HEADLR_MIN_ANGLE 30
#define HEADLR_MAX_ANGLE 90
/**** End Angle Definitions ****/
/**** Accelerometer Thresholds ****/
const int forwardThreshold = 340;
const int backwardThreshold = 330;
const int leftThreshold = 360;
const int rightThreshold = 350;
/**** END Accelerometer Thresholds ****/

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

Thread buttonTh = Thread();
Thread commonMovesTh = Thread();
Thread blinkTh = Thread();
Thread moveHeadTh = Thread();
ThreadController control1 = ThreadController();


// Boolean for second blink control
byte secondBlink = 0;
// Eye blink interval in ms
int blinkinterval = 3000;
// ms to increase the voltage on the heartbeat led
int heartbeatDelay = 50;
// Timer storage
int lastBlinkTime = 0;
int lastEyeMovement = 0;
// Servo pulse widths
int min_pulse = 600;
int max_pulse = 2400;
// Shall we light the heart or dim it
byte invert = 0;
byte brightness = 0;

// Will change if bye boru is cross or excited
byte heartbeatStep = 10;

// Globals to keep the current angle positions
// Used for small eye movements
int curLEyeAngle = INITIAL_EYE_LR_ANGLE;
int curREyeAngle = INITIAL_EYE_LR_ANGLE;

byte headinvert = 0;

// When made true anything else will stop
bool ultimateaction = false;
// when true random eye moves won't run
bool stopeyemoves = false;
// Accelerometer will be disengaged if true
bool ignoreacc = false;
/**** Commands to send to the other arduino
* mouthcommand = 0 Normal operation
* mouthcommand = 1 Open mouth
* mouthcommand = 2 stop everything
* Anything bigger than 7 sets permament mouth angle until 0 is sent
*****/
byte mouthcommand = 0;

// Accelerometer Parameters
//float xZero = 319;
//float yZero = 337;
//float zZero = 419;
//int Scale = 1;
double xAcc = 0;
double yAcc = 0;
double zAcc = 0;
const float alpha = 0.5;

void setup()
{
  // For debugging purposes
  //Serial.begin(9600);
  //SETUP heart
  pinMode(HEART_LED, OUTPUT);
  
  // Make random really random by reading from A2
  randomSeed(analogRead(A2));

  // Thread callback definitions
  heartbeatTh.onRun(heartbeatCallback);
  heartbeatTh.setInterval((heartbeatDelay*10)+10);

  buttonTh.onRun(buttonHandler);
  buttonTh.setInterval((heartbeatDelay*10)+10);

  commonMovesTh.onRun(commonMovesCallback);
  commonMovesTh.setInterval(500);

  blinkTh.onRun(blinkMeEyes);
  blinkTh.setInterval(3000);

  moveHeadTh.onRun(moveHeadCallback);
  moveHeadTh.setInterval(300);

  //thread group definition
  control1.add(&heartbeatTh);
  control1.add(&buttonTh);  
  control1.add(&commonMovesTh);
  control1.add(&blinkTh);
  control1.add(&moveHeadTh);
    
  //Servo setups
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
  
  //servo initial positions
  eyeLidL.write(LEYE_LID_OPEN_ANGLE);
  eyeLidR.write(REYE_LID_OPEN_ANGLE);

  // Servo Arduino communications
  Wire.begin();
}

void loop()
{
  while(1)
    control1.run();
}

/**************** CALLBACK FUNCTIONS *************/
void buttonHandler()
{
  if(ultimateaction == true)
    return;
  int button1Aval = 0, button2Aval = 0;
  byte button1val = 0, button2val = 0; 
  

  button1Aval = analogRead(BUTTON1_INPUT_PIN);
  button2Aval = analogRead(BUTTON2_INPUT_PIN);

  if(button1Aval >= 200 && button1Aval <= 210)
    button1val = 1;
  else if(button1Aval >= 505 && button1Aval <=515)
    button1val = 2;
  else if(button1Aval >= 835 && button1Aval <= 845)
    button1val = 3;
    
  if(button2Aval >= 200 && button2Aval <= 210)
    button2val = 1;
  else if(button2Aval >= 505 && button2Aval <=515)
    button2val = 2;
  else if(button2Aval >= 835 && button2Aval <= 845)
    button2val = 3;

  // This is gonna be ugly
  if(button1val == 0 && button2val == 0) {
    // XXX Handle action when buttons are released
    if(ignoreacc == true) {
      ignoreacc = false;
      // XXX Set new acc values here
    }
  } else if(button1val == 1 && button2val == 0) {
    //L1 single press
    bbEarSpin();
  } else if(button1val == 2 && button2val == 0) {
    //L2 single press
    bbFear();
  } else if(button1val == 3 && button2val == 0) {
    //L3 single press
    bbMouthOpen();
  } else if(button2val == 1 && button1val == 0) {
    //R1 single press
    bbWink();
  } else if(button2val == 2 && button1val == 0) {
    //R2 single press
    bbFrown();
  } else if(button2val == 3 && button1val == 0) {
    //R3 single press
    bbWhistle();
  } else if(button2val == 3 && button1val == 3) {
    //L3 + R3 bb dies
    // No return back from this point
    bbDie();
  } else if(button2val == 1 && button1val == 1) {
    //L1 + R1 searching eyes
    bbSearch();
  } else if(button1val == 1 && button2val == 2) {
    //L1 + R2 I'm the man
    bbimCool();
  } else if(button1val == 1 && button2val == 3) {
    // L1 + R3 reset accelerometer
    //Accelerometer is disengaged, 
    // when button is released, that is the new "zero" position
    ignoreacc = true;
  } else if(button1val == 2 && button2val == 3) {
    //L2 + R3 crazy eyes
    bbCrazyEyes();
  }

}

void heartbeatCallback()
{
   if(invert == 0) {
    for (int i=0; i<heartbeatStep; i++) {
      analogWrite(HEART_LED, brightness);
      brightness += 20;
       delay(heartbeatDelay);
    }
    invert = 1;
  } else {
    for(int i=0; i<heartbeatStep; i++) {
      analogWrite(HEART_LED, brightness);
      brightness -= 20;
      delay(heartbeatDelay);
    }
    invert = 0;
  }  
}

void commonMovesCallback()
{
  if(ultimateaction == true)
    return;
    
  // 50% chance to move eyes every 500 ms
  if(random(100) > 50)
    moveEyesSlightly();
}

void moveHeadCallback()
{
  if(ignoreacc = true)
    return;
  int xAxis = analogRead(ACC_X_INPUT_PIN);
  int yAxis = analogRead(ACC_Y_INPUT_PIN);
  int zAxis = analogRead(ACC_Z_INPUT_PIN);
  //Low Pass Filter
  xAcc = xAxis * alpha + (xAcc * (1.0 - alpha));
  yAcc = yAxis * alpha + (yAcc * (1.0 - alpha));
  zAcc = zAxis * alpha + (zAcc * (1.0 - alpha));
  double roll = (atan2(-zAcc, xAcc)*180.0/PI);

  if(roll <= HEADLR_MAX_ANGLE && roll >= HEADLR_MIN_ANGLE)
    headLR.write(roll);
}

/*********** END CALLBACK FUNCTIONS *******************/

/*********** HELPER/COMMON FUNCTIONS *************************/
// Eye blink operation
void blinkMeEyes()
{
  if(stopeyemoves == true)
    return;
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

/********** end HELPER functions *************/
/********** Facial Expressions ***********/
void bbNeutral()
{
  return;
}

// Both ears spin
// Called when L1 is pressed
void bbEarSpin()
{
  ears.write(EARS_FINAL_ANGLE);
  delay(150);
  ears.write(EARS_INITIAL_ANGLE); 
}

// Open Mouth
// Called when L2 is pressed
void bbMouthOpen()
{
  Wire.beginTransmission(1);
  Wire.write(1);
  Wire.endTransmission();
}

//Mouth and eyes open wide, eye brows up, pointing up
// Called when L2 is pressed
void bbFear()
{

}

//mouth smiles, one eye shut, other eye brow points upwards
// Called when R1 is pressed
void bbWink()
{
  
}

// eyes slightly closed
// Called when R2 is pressed
void bbFrown()
{
  
}

// lips pursed, eye brows low, eyes slightly closed
// Called when R3 is pressed
void bbWhistle()
{
  
}

//heart beat slows, erratic, eyes flicker open and close, 
//finally slowly close, eye brows go high then slowly lower,
// head tilts down - if servo works!
// Called when L3+R3 are pressed
void bbDie()
{
  ultimateaction = true;
  Wire.beginTransmission(1);
  Wire.write(1);
  Wire.endTransmission();
}

// eyes look left and right together
// Called when L1+R1 are pressed
void bbSearch()
{
  stopeyemoves = true;
  delay(200);
  stopeyemoves = false;
}

//smile, eye brows raise 3 times in succession, 
//finish with a single wink
// Called when L1+R2 are pressed
void bbimCool()
{
  
}

void bbCrazyEyes()
{
  // Eyes, eyelids and eyebrows move wildly and independently

}
/*************** END FACIAL EXPRESSIONS *********************/

