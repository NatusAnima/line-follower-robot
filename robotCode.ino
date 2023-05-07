#include <QTRSensors.h>
 

 // the pins for each sensor
#define SENSOR1_PIN A0
#define SENSOR2_PIN A1
#define SENSOR3_PIN A2
#define SENSOR4_PIN A3
#define SENSOR5_PIN A4
#define SENSOR6_PIN A5
#define SENSOR7_PIN A6
#define SENSOR8_PIN A7

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){SENSOR1_PIN, SENSOR2_PIN, SENSOR3_PIN, SENSOR4_PIN, SENSOR5_PIN, SENSOR6_PIN, SENSOR7_PIN, SENSOR8_PIN}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
  // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 13 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin



}

void loop()
{

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 8000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  //This method technically remembers when it last saw the black line, so the value stored in it will stay consistent
  //if we lose the black line

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  int leftMotorSpeed = 255;
  int rightMotorSpeed = 255;
 
  // compute our "error" from the line position.  We will make it so that the error is
  // zero when the middle sensors are over the line, because this is our goal.  Error
  // will range from -4000 to +4000.  If we have sensor 0 on the far left and sensor 7 on
  // the far right,  a reading of -4000 means that we see the line on the very far left and a reading
  // of +4000 means we see the line on the very far right.
  // zero is the sensor 4
  int error = position - 4000;


 if (error < -3000) {  // the line is on the far left
    leftMotorSpeed = 0;
    rightMotorSpeed = 255;
  } else if (error < -2000) {  // the line is on the left
    leftMotorSpeed = 80;
    rightMotorSpeed = 255;
  } else if (error < -1000) {  // the line is slightly on the left
    leftMotorSpeed = 150;
    rightMotorSpeed = 255;
  } else if (error < 0) {  // the line is slightly on the left
    leftMotorSpeed = 200;
    rightMotorSpeed = 255;
  } else if (error > 3000) {  // the line is on the far right
    leftMotorSpeed = 255;
    rightMotorSpeed = 0;
  } else if (error > 2000) {  // the line is on the right
    leftMotorSpeed = 255;
    rightMotorSpeed = 80;
  } else if (error > 1000) {  // the line is slightly on the right
    leftMotorSpeed = 255;
    rightMotorSpeed = 150;
  } else if (error > 0) {  // the line is slightly on the right
    leftMotorSpeed = 255;
    rightMotorSpeed = 200;
  } else {  // the robot is on the line
    leftMotorSpeed = 255;
    rightMotorSpeed = 255;
  }

  //Motor A forward @ full speed
  digitalWrite(12, HIGH);  //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, rightMotorSpeed);    //Spins the motor on Channel A at half speed
  
  //Motor B forward @ full speed
  digitalWrite(13, HIGH); //Establishes forward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, leftMotorSpeed);   //Spins the motor on Channel B at full speed
  

}
