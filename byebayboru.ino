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
  
  PIN 47  -> Eye Lid R
  PIN 4  -> left eye Up-Down
  PIN 3  -> right eye  Up-Down
  PIN 2  -> left eye  L-R
  PIN 48  -> right eye L-R
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
  APIN 0 -> Sound input
  APIN 2 -> Accelerometer X
  APIN 3 -> Accelerometer Y
  APIN 4 -> Accelerometer Z
  APIN 5 -> Button Set 1
  APIN 6 -> Button Set 2
******* END PIN LAYOUTS *********************
*********************************************
******* CONFIGURABLE PARAMETERS *************
*********************************************
******* PIN definitions as stated above ****/
#define R_EYE_UD 3
#define L_EYE_UD 4
#define L_EYE_LR 2
#define R_EYE_LR 48
#define EYE_LID_L 47
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
/**** End Servo PIN definitions ****
***** Audio INPUT PIN definition (Mono) ****/
#define AUDIO_INPUT_PIN A0
/**** Accelerometer INPUTS ****/
#define ACC_X_INPUT_PIN A2
#define ACC_Y_INPUT_PIN A3
#define ACC_Z_INPUT_PIN A4
/***** Button Inputs *******/
#define BUTTON1_INPUT_PIN A5
#define BUTTON2_INPUT_PIN A6
/***** End Button Inputs ******************
*******************************************
***** End INPUT PIN definitions ***********
*******************************************
***** INITIAL Servo Angle Definitions ****/
static byte LEYE_LID_INITIAL_ANGLE = 65;
static byte REYE_LID_INITIAL_ANGLE = 40;
static byte EYEBROW_L_INITIAL_ANGLE = 90;
static byte EYEBROW_R_INITIAL_ANGLE = 95;
static byte LEYE_LR_INITIAL_ANGLE = 90;
static byte REYE_LR_INITIAL_ANGLE = 90;
static byte LEYE_UD_INITIAL_ANGLE = 90;
static byte REYE_UD_INITIAL_ANGLE = 90;
static byte EYEBROW_UD_INITIAL_ANGLE = 100;

byte LEYE_LID_OPEN_ANGLE = 65;
byte LEYE_LID_CLOSE_ANGLE = 85;
byte REYE_LID_OPEN_ANGLE = 40;
byte REYE_LID_CLOSE_ANGLE = 60;
byte MOUTHUD_MIN_ANGLE = 95;
byte MOUTHUD_MAX_ANGLE = 125;
byte MOUTHL_NEUTRAL_ANGLE = 95;
byte MOUTHR_NEUTRAL_ANGLE = 92;
byte MOUTHL_SMILE_ANGLE = 45;
byte MOUTHR_SMILE_ANGLE = 120;
byte MOUTHL_FROWN_ANGLE = 120;
byte MOUTHR_FROWN_ANGLE = 45;
byte INITIAL_EYE_LR_ANGLE = 90;
byte EARS_INITIAL_ANGLE = 90;
byte EARS_FINAL_ANGLE = 120;
byte HEADLR_MIN_ANGLE = 30;
byte HEADLR_MAX_ANGLE = 90;
byte mouth_ud_step = 5;

/**** End Angle Definitions ********
***** Accelerometer Thresholds ****/
const int forwardThreshold = 340;
const int backwardThreshold = 330;
const int leftThreshold = 360;
const int rightThreshold = 350;
/**** END Accelerometer Thresholds ****
****** End Configurable Stuff *********/
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
Thread talkTh = Thread();
ThreadController control1 = ThreadController();


// Boolean for second blink control
byte secondBlink = 0;
// Eye blink interval in ms
int blinkinterval = 3000;
// ms to increase the voltage on the heartbeat led
int heartbeatDelay = 50;
int heartbeatDelaytmp = heartbeatDelay;
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

// Accelerometer will be disengaged if true
bool ignoreacc = false;
/**** Face expression state  variables ****/
bool mouthopen = false;
bool fear = false;
bool wink = false;
bool frown = false;
bool whistle = false;
bool search = false;
bool crazy = false;
bool lie = false;
bool stoptalking = false;
bool stopeyes = false;
bool stopblinking = false;
bool smile = false;
bool anger = false;
bool bigsmile = false;
bool thinking = false;
bool notsure = false;
bool raiseeyebrows = false;
bool stopheart = false;
/**** End Face expression state variables ****/
// Accelerometer Parameters
//float xZero = 319;
//float yZero = 337;
//float zZero = 419;
//int Scale = 1;
double xAcc = 0;
double yAcc = 0;
double zAcc = 0;
const float alpha = 1.5;

// Make this false to stop random eyemovements
bool enableeyemovements = true;

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
  buttonTh.setInterval(500);



  blinkTh.onRun(blinkMeEyes);
  blinkTh.setInterval(3000);

  moveHeadTh.onRun(moveHeadCallback);
  moveHeadTh.setInterval(200);

  talkTh.onRun(talkCallback);
  talkTh.setInterval(200);
  
  if(enableeyemovements == true) {
    commonMovesTh.onRun(commonMovesCallback);
    commonMovesTh.setInterval(500);
    control1.add(&commonMovesTh);
  }
  
  //thread group definition
  control1.add(&heartbeatTh);
  control1.add(&buttonTh);  
  control1.add(&blinkTh);
  control1.add(&talkTh);
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
  mouthUD.attach(MOUTH_UD, min_pulse, max_pulse);
  headUD.attach(HEAD_UD, min_pulse, max_pulse);
  headLR.attach(HEAD_LR, min_pulse, max_pulse);
  eyeBrowR.attach(EYE_BROW_R, min_pulse, max_pulse);
  eyeBrowL.attach(EYE_BROW_L, min_pulse, max_pulse);
  ears.attach(EARS, min_pulse, max_pulse);
  eyeBrowU.attach(EYE_BROW_UD, min_pulse, max_pulse);
  
  //servo initial positions
  bbNeutral();

  Serial.begin(9600);
}

void loop()
{
  while(1)
    control1.run();
}

/**************** CALLBACK FUNCTIONS *************/
void talkCallback()
{
  if(stoptalking == true)
    return;
    
  int audioVal, outputVal;
  audioVal = analogRead(AUDIO_INPUT_PIN);
  outputVal = abs(map(audioVal, 520, 1023, MOUTHUD_MIN_ANGLE, MOUTHUD_MAX_ANGLE));  
  Serial.println(outputVal); 
  mouthUD.write(outputVal); 
}

void buttonHandler()
{
  if(ultimateaction == true) //Bye boru's dying just return 
    return;
  
  int button1Aval = 0, button2Aval = 0;
  byte button1val = 0, button2val = 0; 

  button1Aval = analogRead(BUTTON1_INPUT_PIN);
  delay(200);
  button2Aval = analogRead(BUTTON2_INPUT_PIN);

  //Map analog values to buttons 1,2,3
  if(button1Aval >= 150 && button1Aval <= 230)
    button1val = 1;
  else if(button1Aval >= 400 && button1Aval <= 600)
    button1val = 2;
  else if(button1Aval >= 750 && button1Aval <= 900)
    button1val = 3;
    
  if(button2Aval >= 150 && button2Aval <= 230)
    button2val = 1;
  else if(button2Aval >= 400 && button2Aval <= 600)
    button2val = 2;
  else if(button2Aval >= 750 && button2Aval <= 900)
    button2val = 3;

  // This is gonna be ugly
  // These are the button combinations
  if(button1val == 0 && button2val == 0) {
    // Handle action when buttons are released
    // Serial.println("resetting face...");
    if(ignoreacc == true) {
      ignoreacc = false;
      xAcc = 0;
      zAcc = 0;
    }
    if(whistle == true)
      bbWhistle(false);
    
    if(anger == true)
      bbAnger(false);
      
    if(bigsmile == true)
      bbBigSmile(false);
      
    if(mouthopen == true)
      bbMouthOpen(false);
   
    if(fear == true)
      bbFear(false); 

    if(wink == true)
      bbWink(false);

    if(frown == true)
      bbFrown(false);
      
    if(search == true)
      bbSearch(false);

    if(thinking == true)
      bbThink(false);
      
    if(notsure == true)
      bbNotSure(false);
      
    if(mouthopen == true)
      bbMouthOpen(false);
    if(crazy == true) {
      crazy = false;
      bbNeutral();
    }

    if(lie == true) {
      lie = false;
      bbLie(false);
    }
  } else if(button1val == 3 && button2val == 3) {
    //L3 + R3 bb dies
    // No return back from this point
    bbDie();
  } else if(button1val == 1 && button2val == 1) {
    //L1 + R1 searching eyes
    bbSearch(true);
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
  } else if(button1val == 2 && button2val == 2) {
    //L2 + R2 Neutral face
    bbNeutral();
  } else if(button1val == 2 && button2val == 1) {
    //L2 + R1 Big smile
    if(bigsmile == true)
      bbBigSmile(true);
  } else if(button1val == 3 && button2val == 1) {
    if(smile == true)
      bbSmile(true);
  } else if(button1val == 3 && button2val == 2) {
      bbRaiseEyeB();
  } else if(button1val == 1 && button2val == 0) {
    //L1 single press
    if(anger == false)
      bbAnger(true);
  } else if(button1val == 2 && button2val == 0) {
    //L2 single press
    // Don't try to exec again
    if(fear == false)
      bbFear(true);
  } else if(button1val == 3 && button2val == 0) {
    //L3 single press
    if(mouthopen == false)
      bbMouthOpen(true);
  } else if(button2val == 1 && button1val == 0) {
    //R1 single press
    if(thinking == false)
      bbThink(true);
  } else if(button2val == 2 && button1val == 0) {
    //R2 single press
    if(notsure == false)
      bbNotSure(true);
  } else if(button2val == 3 && button1val == 0) {
    //R3 single press
    if(whistle == false)
      bbWhistle(true);
  }

}

void heartbeatCallback()
{
  if(stopheart == true) {
    analogWrite(HEART_LED, 0);
    return;
  }
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
  if(ignoreacc == true)
    return;
  
  int xAxis = analogRead(ACC_X_INPUT_PIN);
  //int yAxis = analogRead(ACC_Y_INPUT_PIN);
  int zAxis = analogRead(ACC_Z_INPUT_PIN);
  //Low Pass Filter
  xAcc = xAxis * alpha + (xAcc * (1.0 - alpha));
  //yAcc = yAxis * alpha + (yAcc * (1.0 - alpha));
  zAcc = zAxis * alpha + (zAcc * (1.0 - alpha));
  double roll = (atan2(zAcc, xAcc)*180.0/PI);
  //Serial.println(roll);
  if(roll <= HEADLR_MAX_ANGLE && roll >= HEADLR_MIN_ANGLE)
    headLR.write(roll+30);
}

/*********** END CALLBACK FUNCTIONS *******************/

/*********** HELPER/COMMON FUNCTIONS *************************/
// Eye blink operation
void blinkMeEyes()
{
  if(stopblinking == true || ultimateaction == true)
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
  if(stopeyes == true || ultimateaction == true)
    return;
  int curRndNum = random(-20,0);

  if(curLEyeAngle < 20 || curREyeAngle < 20)
	  return;

  curLEyeAngle = curLEyeAngle - curRndNum;
  curREyeAngle = curREyeAngle + curRndNum;
  lEyeLR.write(curLEyeAngle);
  rEyeLR.write(curREyeAngle);

  delay(100);

  curLEyeAngle = curLEyeAngle - abs(curRndNum);
  curREyeAngle = curREyeAngle + abs(curRndNum);
  lEyeLR.write(curLEyeAngle);
  rEyeLR.write(curREyeAngle);
}

/********** end HELPER functions *************/
/********** Facial Expressions ***********/
void bbNeutral()
{
  mouthUD.write(5);
  lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
  rEyeLR.write(REYE_LR_INITIAL_ANGLE);
  lEyeUD.write(LEYE_UD_INITIAL_ANGLE);
  rEyeUD.write(REYE_UD_INITIAL_ANGLE);
  eyeLidL.write(LEYE_LID_OPEN_ANGLE);
  eyeLidR.write(REYE_LID_OPEN_ANGLE);
  eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
  eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
  eyeBrowU.write(EYEBROW_UD_INITIAL_ANGLE);
  mouthL.write(MOUTHL_NEUTRAL_ANGLE);
  mouthR.write(MOUTHR_NEUTRAL_ANGLE);
  stoptalking = false;
  stopblinking = false;
}

//Eyebrows raised, point down,
//lips down (frown)
// Called when L1 is pressed
void bbAnger(bool fstatus)
{
  if(fstatus == true && anger == false) {
    anger = true;
    mouthL.write(MOUTHL_FROWN_ANGLE);
    mouthR.write(MOUTHR_FROWN_ANGLE);
    eyeBrowL.write(60);
    eyeBrowR.write(115);
    eyeBrowU.write(75);
    Serial.println("BB angry");
  } else if(fstatus == false && anger == true) {
    anger = false;
    eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
    eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
    eyeBrowU.write(EYEBROW_UD_INITIAL_ANGLE);
    mouthL.write(MOUTHL_NEUTRAL_ANGLE);
    mouthR.write(MOUTHR_NEUTRAL_ANGLE);
    Serial.println("BB not angry");
  }
  
}

//Eyes look up to the left, eye balls 60% open
// called when R1 is pressed
void bbThink(bool fstatus)
{
  if(fstatus == true && thinking == false) {
    thinking = true;
    LEYE_LID_OPEN_ANGLE = 60;
    REYE_LID_OPEN_ANGLE = 35;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    lEyeUD.write(LEYE_UD_INITIAL_ANGLE-20);
    rEyeUD.write(REYE_UD_INITIAL_ANGLE-20);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE-20);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE-20);
    Serial.println("BB thinking");
  } else if(fstatus == false && thinking == true) {
    thinking = false;
    LEYE_LID_OPEN_ANGLE = LEYE_LID_INITIAL_ANGLE;
    REYE_LID_OPEN_ANGLE = REYE_LID_INITIAL_ANGLE;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    lEyeUD.write(LEYE_UD_INITIAL_ANGLE);
    rEyeUD.write(REYE_UD_INITIAL_ANGLE);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE);
    Serial.println("BB not thinking");
  }
}

//Open mouth, smile, eyebrows up, pointing up
// Called when L2 + R1 is pressed
void bbBigSmile(bool fstatus)
{
  if(fstatus == true && bigsmile == false) {
    bigsmile = true;
    bbMouthOpen(true);
    eyeBrowU.write(75);
    eyeBrowL.write(125);
    eyeBrowR.write(65);
    bbSmile(true);
    Serial.println("BB big smile");
  } else if(fstatus == false && bigsmile == true) {
    bigsmile = false;
    bbMouthOpen(false);
    bbSmile(false);
    eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
    eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
    eyeBrowU.write(EYEBROW_UD_INITIAL_ANGLE);
    Serial.println("BB no big smile");
  }
  
}

//Eyebrows down, pointing down, 
//eyes slowly look left and right,
//while looking down
// Called when R2 is pressed
void bbNotSure(bool fstatus)
{
  int tempvar;
  if(fstatus == true && notsure == false) {
    notsure = true;
    //eyeBrowU.write(45);
    eyeBrowL.write(60);
    eyeBrowR.write(115);
    for(int i=90;i>59;i=i-10) {
      lEyeLR.write(i);
      rEyeLR.write(i);
      delay(20);
    }
    delay(50);
    for(int i=60;i<121;i=i+10) {
      rEyeLR.write(i);
      lEyeLR.write(i);
      delay(20);
    }
    Serial.println("BB not sure");
  } else if(fstatus == false && notsure == true) {
    notsure = false;
    eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
    eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE);
    //eyeBrowU.write(EYEBROW_UD_INITIAL_ANGLE);
    Serial.println("BB sure");
  }
}

//Eyebrows keep going up and down, at the top, they twitch
void bbRaiseEyeB()
{
  Serial.println("BB raise eyes");
  eyeBrowU.write(75); 
  eyeBrowL.write(80);
  eyeBrowR.write(80);
  delay(50);
  eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
  eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
  delay(50);
  eyeBrowL.write(80);
  eyeBrowR.write(80);
  delay(150);
  eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
  eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
  eyeBrowU.write(EYEBROW_UD_INITIAL_ANGLE);
}
// Both ears spin
void bbEarSpin()
{
  ears.write(EARS_FINAL_ANGLE);
  delay(150);
  ears.write(EARS_INITIAL_ANGLE); 
}

// Open Mouth
// Called when L2 is pressed
void bbMouthOpen(bool fstatus)
{
  if(fstatus == true && mouthopen == false) {
    mouthopen = true;
    stoptalking = true;
    mouthUD.write(120);
  } else if(fstatus == false && mouthopen == true) {
    mouthopen = false;
    mouthUD.write(95);
    stoptalking = false;
  }
}

//Mouth and eyes open wide, eye brows up, pointing up
// Called when L2 is pressed
void bbFear(bool fstatus)
{
  if(fstatus == true && fear == false) {
    fear = true;
    stoptalking = true;
    heartbeatDelay = heartbeatDelay/2;
    // Set open angles to new values 
    // so we can continue blinking
    LEYE_LID_OPEN_ANGLE = 50;
    REYE_LID_OPEN_ANGLE = 30;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    mouthUD.write(120);
    eyeBrowL.write(125);
    eyeBrowR.write(65);
    Serial.println("BB in fear");
  } else if(fstatus == false && fear == true) {
    heartbeatDelay = heartbeatDelaytmp;
    fear = false;
    stoptalking = false;
    LEYE_LID_OPEN_ANGLE = LEYE_LID_INITIAL_ANGLE;
    REYE_LID_OPEN_ANGLE = REYE_LID_INITIAL_ANGLE;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
    eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
    Serial.println("BB not in fear");
  }
}

//mouth smiles, one eye shut, other eye brow points upwards
// Called when R1 is pressed
void bbWink(bool fstatus)
{
  if(fstatus == true && wink == false) {
    wink = true;
    stopblinking = true;
    eyeLidR.write(REYE_LID_CLOSE_ANGLE);
    eyeBrowL.write(115);
    bbSmile(true);
  } else if(fstatus == false && wink == true) {
    wink = false;
    stopblinking = false;
    eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    bbSmile(false);
  }
  
}

// lips pursed, eye brows low, eyes slightly closed
// Called when R3 is pressed
void bbWhistle(bool fstatus)
{
  if(fstatus == true && whistle == false) {
    whistle = true;
    stoptalking = true;
    // Set open angles to new values 
    // so we can continue blinking
    LEYE_LID_OPEN_ANGLE = 70;
    REYE_LID_OPEN_ANGLE = 50;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    mouthUD.write(100);
    Serial.println("BB whistles");
  } else if(fstatus == false && whistle == true) {
    whistle = false;
    stoptalking = false;
    LEYE_LID_OPEN_ANGLE = LEYE_LID_INITIAL_ANGLE;
    REYE_LID_OPEN_ANGLE = REYE_LID_INITIAL_ANGLE;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
    eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
    Serial.println("BB doesn't whistle");
  }
  
}

//heart beat slows, erratic, eyes flicker open and close, 
//finally slowly close, eye brows go high then slowly lower,
// head tilts down - if servo works!
// Called when L3+R3 are pressed
void bbDie()
{
  ultimateaction = true;
  heartbeatDelay = heartbeatDelay*2;
  // Close mouth
  mouthUD.write(5);
  // Flicker eyes
  for(int i=0; i<5; i++) {
    lEyeLR.write(60);
    rEyeLR.write(60);
    eyeLidL.write(LEYE_LID_CLOSE_ANGLE);
    eyeLidR.write(REYE_LID_CLOSE_ANGLE);
    lEyeLR.write(90);
    rEyeLR.write(90);
    delay(50);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    lEyeLR.write(120);
    rEyeLR.write(120);
  }

  // Slowly close eyes
  for(int i=LEYE_LID_OPEN_ANGLE,j=REYE_LID_OPEN_ANGLE;i<=REYE_LID_OPEN_ANGLE && j<=REYE_LID_OPEN_ANGLE;i--,j--) {
    eyeLidR.write(j);
    eyeLidL.write(i);
    delay(50);
  }  
  
  // Eye brows go up and slowly down
  eyeBrowU.write(75);
  for(int i=75;i>=EYEBROW_UD_INITIAL_ANGLE; i--) {
    eyeBrowU.write(i);
    delay(100);
  }
  stopheart = true;
}

// eyes look left and right together
// Called when L1+R1 are pressed
void bbSearch(bool fstatus)
{
  if(fstatus == true && search == false) {
    stopeyes = true;
    search = true;
    lEyeLR.write(60);
    rEyeLR.write(60);
    delay(300);
    lEyeLR.write(120);
    rEyeLR.write(120);
    delay(300);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE);
    Serial.println("BB searches");
  } else if(fstatus == false && search == true) {
    stopeyes = false;
    search = false;
    Serial.println("BB doesn't search");
  } 
}

//smile, eye brows raise 3 times in succession, 
//finish with a single wink
// Called when L1+R2 are pressed
void bbimCool()
{
    stopeyes = true;
    bbSmile(true);
    for(int i=0;i<3;i++) {
      eyeBrowU.write(75);
      delay(150);
      eyeBrowU.write(EYEBROW_UD_INITIAL_ANGLE);
    }
    eyeLidR.write(REYE_LID_CLOSE_ANGLE);
    delay(150);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    delay(50);
    bbSmile(false);
    stopeyes = false;
    Serial.println("BB I'm cool!");
}

// Eyes, eyelids and eyebrows 
// move wildly and independently
// Called when L2+R3 are pressed
void bbCrazyEyes()
{
  crazy = true;
  stopblinking = true;
  stoptalking = true;
  byte leyeangle, reyeangle, leyerlangle, reyerlangle;
  byte leyeudangle, reyeudangle, leyebangle, reyebangle;
  byte leyelangle, reyelangle;
  // Eyes, eyelids and eyebrows move wildly and independently
  leyerlangle = random(40,140);
  //reyerlangle = random(40,150);
  lEyeLR.write(leyerlangle);
  rEyeLR.write(leyerlangle*-1);
  leyeudangle = random(40,140);
  //reyeudangle = random(40,150);
  lEyeUD.write(leyeudangle);
  rEyeUD.write(leyeudangle*-1);
  //leyelangle = random(LEYE_LID_OPEN_ANGLE-20, LEYE_LID_CLOSE_ANGLE);
  //reyelangle = random(REYE_LID_OPEN_ANGLE-20, REYE_LID_CLOSE_ANGLE);
  eyeLidL.write(LEYE_LID_OPEN_ANGLE-10);
  eyeLidR.write(REYE_LID_OPEN_ANGLE-5);
  leyebangle = random(60, 120);
  reyebangle = random(60, 120);
  eyeBrowL.write(leyebangle);
  eyeBrowR.write(reyebangle);
  bbEarSpin();
  Serial.println("I'm going slightly mad!");
}

// If true => then smile, false => then be neutral
// Called when L3+R1 is pressed
void bbSmile(bool fstatus)
{
  if(fstatus == true && smile == false) {
    smile = true;
    stoptalking = true;
    mouthR.write(MOUTHR_SMILE_ANGLE);
    mouthL.write(MOUTHL_SMILE_ANGLE);
    Serial.println("BB small smile");
  } else if(fstatus == false && smile == true) {
    smile = false;
    stoptalking = false;
    mouthL.write(MOUTHL_NEUTRAL_ANGLE);
    mouthR.write(MOUTHR_NEUTRAL_ANGLE);
    Serial.println("BB don't smile");
  }
}

// eyes look left, then right together, 
// but are looking down
// Not called
void bbLie(bool fstatus)
{
  if(fstatus == true && lie == false) {
    lie = true;
    lEyeLR.write(60);
    rEyeLR.write(60);
    lEyeUD.write(LEYE_UD_INITIAL_ANGLE-20);
    rEyeUD.write(REYE_UD_INITIAL_ANGLE-20);
    delay(200);
    lEyeLR.write(120);
    rEyeLR.write(120);
    delay(200);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE);
  } else if(fstatus == false && lie == true) {
    lie = false;
    lEyeUD.write(LEYE_UD_INITIAL_ANGLE);
    rEyeUD.write(REYE_UD_INITIAL_ANGLE);
  }
}



// eyes slightly closed
// Not called currently
void bbFrown(bool fstatus)
{
  if(fstatus == true && frown == false) {
    LEYE_LID_OPEN_ANGLE = 60;
    REYE_LID_OPEN_ANGLE = 40;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
  } else if(fstatus == false && frown == true) {
    LEYE_LID_OPEN_ANGLE = LEYE_LID_INITIAL_ANGLE;
    REYE_LID_OPEN_ANGLE = REYE_LID_INITIAL_ANGLE;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
  }
}
/*************** END FACIAL EXPRESSIONS *********************/

