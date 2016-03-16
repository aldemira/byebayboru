const int forwardThreshold = 340;
const int backwardThreshold = 330;
const int leftThreshold = 360;
const int rightThreshold = 350;
#define ACC_X_INPUT_PIN A8
#define ACC_Y_INPUT_PIN A10
#define ACC_Z_INPUT_PIN A12

float xZero = 319;
float yZero = 337;
float zZero = 419;
int Scale = 1;

void setup()
{
  Serial.begin(9600);
	
}

void loop()
{
	int xAxis = analogRead(ACC_X_INPUT_PIN);
	int yAxis = analogRead(ACC_Y_INPUT_PIN);
	int zAxis = analogRead(ACC_Z_INPUT_PIN);
	Serial.print("x: ");
	Serial.print(xAxis);
	Serial.print(" y: ");
	Serial.print(yAxis);
	Serial.print(" z: ");
	Serial.println(zAxis);
  float xAcc = (xAxis - xZero)/Scale;
  float yAcc = (yAxis - yZero)/Scale;
  float zAcc = (zAxis - zZero)/Scale;
  float pitch = atan(xAcc/sqrt(pow(yAcc,2) + pow(zAcc,2)));
  float roll = atan(yAcc/sqrt(pow(xAcc,2) + pow(zAcc,2)));
  //convert radians into degrees
  pitch = pitch * (180.0/PI);
  roll = roll * (180.0/PI) ;
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print(" roll: ");
  Serial.println(roll);
	delay(1500);
}
