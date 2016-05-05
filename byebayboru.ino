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
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
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
byte REYE_LID_CLOSE_ANGLE = 20;
byte MOUTHUD_MIN_ANGLE = 95;
byte MOUTHUD_MAX_ANGLE = 125;
byte MOUTHL_NEUTRAL_ANGLE = 80;
byte MOUTHR_NEUTRAL_ANGLE = 107;
byte MOUTHL_SMILE_ANGLE = 55;
byte MOUTHR_SMILE_ANGLE = 125;
byte MOUTHL_FROWN_ANGLE = 135;
byte MOUTHR_FROWN_ANGLE = 45;
byte INITIAL_EYE_LR_ANGLE = 90;
byte EARS_INITIAL_ANGLE = 90;
byte EARS_FINAL_ANGLE = 120;
byte HEADLR_MIN_ANGLE = 30;
byte HEADLR_MAX_ANGLE = 90;
byte mouth_ud_step = 5;

/**** End Angle Definitions ********
***** Accelerometer/Gyroscope Definitions ****/
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

/**** END Accelerometer/Gyroscope definitons ****/

const int SILENT_AUDIO_VAL = 500;
/****** End Configurable Stuff *********/

void(* resetFunc) (void) = 0; //declare reset function at address 0
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
bool iamcool = false;
bool freeze = false;
/**** End Face expression state variables ****/

// Make this false to stop random eyemovements
bool enableeyemovements = true;
// Enable/Disable head movements
bool enabledacccode = false;

void setup()
{
  // For debugging purposes
  Serial.begin(9600);
  /************* Accelerometer / Gyroscope Initialization **************/  
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
// initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
*/
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  /********** End Accelerometer / Gyroscope Initialization ************/

  //SETUP heart
  pinMode(HEART_LED, OUTPUT);
  
  // Make random really random by reading from A2
  randomSeed(analogRead(A2));

  // Thread callback definitions
  heartbeatTh.onRun(heartbeatCallback);
  heartbeatTh.setInterval((heartbeatDelay*10)+10);

  buttonTh.onRun(buttonHandler);
  buttonTh.setInterval(300);

  blinkTh.onRun(blinkMeEyes);
  blinkTh.setInterval(3000);

  talkTh.onRun(talkCallback);
  talkTh.setInterval(200);
    
  if(enabledacccode == true) {
    moveHeadTh.onRun(moveHeadCallback);
    moveHeadTh.setInterval(200);
    control1.add(&moveHeadTh);
  }

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
}

void loop()
{
  while(1)
    control1.run();
}

/**************** CALLBACK FUNCTIONS *************/
void talkCallback()
{
  if(stoptalking == true || ultimateaction == true)
    return;
    
  int audioVal, outputVal;
  audioVal = analogRead(AUDIO_INPUT_PIN);
  audioVal = abs(audioVal - SILENT_AUDIO_VAL);
  if (audioVal > 20) {
    // range of useful audioVal seems to go from between 20 and 300
    outputVal = map(audioVal, 20, 300, MOUTHUD_MIN_ANGLE + 10, MOUTHUD_MAX_ANGLE);
    // make sure the mouth angle can't go beyond the MIN and MAX angles so that we don't break a servo or the mouth
    outputVal = constrain(outputVal, MOUTHUD_MIN_ANGLE, MOUTHUD_MAX_ANGLE);
  } else
    outputVal = MOUTHUD_MIN_ANGLE;

  Serial.print("abs(audioVal): ");
  Serial.println(audioVal); 
  Serial.print("outputVal: ");
  Serial.println(outputVal); 
  Serial.println("-----"); 
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
    }
    
    // Don't try to reset servos if we're trying to be cool
    if(iamcool == true)
      return;

    if(freeze == true) {
      freeze = false;
      ultimateaction = false;
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

    if(thinking == true)
      bbThink(false);
      
    if(notsure == true)
      bbNotSure(false);
      
    if(mouthopen == true)
      bbMouthOpen(false);
      
    if(smile == true)
      bbSmile(false);
    if(crazy == true || search == true) {
      crazy = false;
      search = false;
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
    bbSearch();
  } else if(button1val == 1 && button2val == 2) {
    //L1 + R2 I'm the man
    bbimCool();
  } else if(button1val == 1 && button2val == 3) {
    // L1 + R3 reset accelerometer
    //Accelerometer is disengaged, 
    // when button is released, that is the new "zero" position
    ignoreacc = true;
    resetFunc();
  } else if(button1val == 2 && button2val == 3) {
    //L2 + R3 crazy eyes
    bbCrazyEyes();
  } else if(button1val == 2 && button2val == 2) {
    //L2 + R2 Neutral face
    bbNeutral();
  } else if(button1val == 2 && button2val == 1) {
    //L2 + R1 Big smile
    if(bigsmile == false)
      bbBigSmile(true);
  } else if(button1val == 3 && button2val == 1) {
    if(smile == false)
      bbSmile(true);
  } else if(button1val == 3 && button2val == 2) {
    if(freeze == false) {
      freeze = true;
      ultimateaction = true;
    }
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
  if(ignoreacc == true || ultimateaction == true)
    return;
  
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (fifoCount < packetSize)
      return;

    // reset interrupt flag and get INT_STATUS byte
    //mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /* Head UP down
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            int tilt = ypr[1] * 180/M_PI;
            Serial.print(tilt);
            headUD.write(tilt+90);   // Rotation around Y
            */
            Serial.print("\t");
            int roll = ypr[2]*180/M_PI;
            Serial.println(roll);
            if(roll <= HEADLR_MAX_ANGLE && roll >= HEADLR_MIN_ANGLE)
              headLR.write(roll+30);   // Rotation around X
        #endif

    }
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
  mouthUD.write(MOUTHUD_MIN_ANGLE);
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
    heartbeatStep = heartbeatStep / 2;
    mouthL.write(MOUTHL_FROWN_ANGLE);
    mouthR.write(MOUTHR_FROWN_ANGLE);
    eyeBrowL.write(60);
    eyeBrowR.write(135);
    eyeBrowU.write(75);
    Serial.println("BB angry");
  } else if(fstatus == false && anger == true) {
    anger = false;
    heartbeatStep = heartbeatStep * 2;
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
    stopblinking = true;
    stopeyes = true;
    LEYE_LID_OPEN_ANGLE = 70;
    REYE_LID_OPEN_ANGLE = 35;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    lEyeUD.write(LEYE_UD_INITIAL_ANGLE-20);
    rEyeUD.write(REYE_UD_INITIAL_ANGLE-20);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE+20);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE-20);
    Serial.println("BB thinking");
  } else if(fstatus == false && thinking == true) {
    thinking = false;
    stopblinking = false;
    stopeyes = false;
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
    eyeBrowR.write(120);
    lEyeUD.write(LEYE_UD_INITIAL_ANGLE-20);
    rEyeUD.write(REYE_UD_INITIAL_ANGLE-20);
    for(int i=LEYE_LR_INITIAL_ANGLE, j=REYE_LR_INITIAL_ANGLE;i>59 && j<59;i=i-10, j=j-10) {
      lEyeLR.write(i);
      rEyeLR.write(j);
      delay(50);
    }
    delay(100);
    for(int i=60, j=60;i<=LEYE_LR_INITIAL_ANGLE && j>=REYE_LR_INITIAL_ANGLE ;i=i+10, j=j+10) {
      rEyeLR.write(j);
      lEyeLR.write(i);
      delay(100);
    }
    Serial.println("BB not sure");
  } else if(fstatus == false && notsure == true) {
    notsure = false;
    eyeBrowL.write(EYEBROW_L_INITIAL_ANGLE);
    eyeBrowR.write(EYEBROW_R_INITIAL_ANGLE);
    lEyeUD.write(LEYE_UD_INITIAL_ANGLE);
    rEyeUD.write(REYE_UD_INITIAL_ANGLE);
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
  eyeBrowU.write(70); 
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
    heartbeatStep = heartbeatStep / 2;
    // Set open angles to new values 
    // so we can continue blinking
    LEYE_LID_OPEN_ANGLE = 50;
    REYE_LID_OPEN_ANGLE = 50;
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    mouthUD.write(120);
    eyeBrowL.write(125);
    eyeBrowR.write(65);
    Serial.println("BB in fear");
  } else if(fstatus == false && fear == true) {
    heartbeatStep = heartbeatStep * 2;
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
  // Close mouth
  mouthUD.write(MOUTHUD_MIN_ANGLE);
  // Slower heart
  heartbeatStep = heartbeatStep * 2;
  // Flicker eyes
  Serial.println("flicker eyes");
  for(int i=0; i < 3; i++) {
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE-30);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE+30);
    delay(400);
    eyeLidL.write(LEYE_LID_CLOSE_ANGLE);
    eyeLidR.write(REYE_LID_CLOSE_ANGLE);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE);
    delay(400);
    
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    eyeLidL.write(LEYE_LID_OPEN_ANGLE);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE+30);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE-30);
  }
  // Even slower
  Serial.println("slow heart");
  heartbeatStep = heartbeatStep * 2;
  // Eyes back to normal position
  // Serial.println("LR eyes back");
  lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
  rEyeLR.write(REYE_LR_INITIAL_ANGLE);
  delay(200);

  // Eye brows go up and slowly down
  //Serial.println("raise and rhen lower eye brows");
  eyeBrowU.write(75);
  for (int i = 75;i <= EYEBROW_UD_INITIAL_ANGLE; i = i + 4) {
    Serial.println(i);
    eyeBrowU.write(i);
    delay(250);
  }
  
  heartbeatStep = heartbeatStep * 2;
  
  // Slowly close eyes
  // Serial.println("Slowly close eyes");
  int steps_to_close = 10;
  int LEYE_CLOSE_INCREMENT = abs(LEYE_LID_OPEN_ANGLE - LEYE_LID_CLOSE_ANGLE) / steps_to_close;
  int REYE_CLOSE_INCREMENT = abs(REYE_LID_OPEN_ANGLE - REYE_LID_CLOSE_ANGLE) / steps_to_close;

  for (int i = 0; i <= steps_to_close; i++) {
    eyeLidR.write(REYE_LID_OPEN_ANGLE - (i * REYE_CLOSE_INCREMENT));
    eyeLidL.write(LEYE_LID_OPEN_ANGLE + (i * LEYE_CLOSE_INCREMENT));
    delay(300);
  }

  stopheart = true;
}
// eyes look left and right together
// Called when L1+R1 are pressed
void bbSearch()
{
    stopeyes = true;
    search = true;
    lEyeLR.write(60);
    rEyeLR.write(120);
    delay(300);
    lEyeLR.write(120);
    rEyeLR.write(60);
    delay(300);
    lEyeLR.write(LEYE_LR_INITIAL_ANGLE);
    rEyeLR.write(REYE_LR_INITIAL_ANGLE);
    Serial.println("BB searches");
    stopeyes = false;
}

//smile, eye brows raise 3 times in succession, 
//finish with a single wink
// Called when L1+R2 are pressed
void bbimCool()
{
    iamcool = true;
    stopeyes = true;
    stopblinking = true;
    bbSmile(true);
    for(int i=0;i<3;i++) {
      eyeBrowU.write(75);
      delay(150);
      eyeBrowU.write(EYEBROW_UD_INITIAL_ANGLE);
    }
    eyeLidR.write(REYE_LID_CLOSE_ANGLE);
    delay(300);
    eyeLidR.write(REYE_LID_OPEN_ANGLE);
    delay(50);
    bbSmile(false);
    stopeyes = false;
    stopblinking = false;
    iamcool = false;
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
  bbMouthOpen(true);
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

