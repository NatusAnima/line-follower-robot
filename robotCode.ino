#include <QTRSensors.h>
#include <NewPing.h>
#include <SparkFun_APDS9960.h>

// The setup phase of this example calibrates the sensors for ten seconds and
// turns on the Arduino's LED (usually on pin 13) while calibration is going
// on. During this phase, you should expose each reflectance sensor to the
// lightest and darkest readings they will encounter. For example, if you are
// making a line follower, you should slide the sensors across the line during
// the calibration phase so that each sensor can get a reading of how dark the
// line is and how light the ground is. Improper calibration will result in
// poor readings.
//
// The main loop of the example reads the calibrated sensor values and uses
// them to estimate the position of a line. You can test this by taping a piece
// of 3/4" black electrical tape to a piece of white paper and sliding the
// sensor across it. It prints the sensor values to the serial monitor as
// numbers from 0 (maximum reflectance) to 1000 (minimum reflectance) followed
// by the estimated location of the line as a number from 0 to 8000. 1000 means
// the line is directly under sensor 1, 2000 means directly under sensor 2,
// etc. 0 means the line is directly under sensor 0 or was last seen by sensor
// 0 before being lost. 5000 means the line is directly under sensor 5 or was
// last seen by sensor 5 before being lost.


// the pins for each ir sensor
#define SENSOR1_PIN A5
#define SENSOR2_PIN A4
#define SENSOR3_PIN A3
#define SENSOR4_PIN A2
#define SENSOR5_PIN A1
#define SENSOR6_PIN A0
#define SENSOR7_PIN 6
#define SENSOR8_PIN 5

#define SENSOR_9960 5  // sensor ADPS-9960

// Ultrasonic
#define TRIGGER_PIN 4
#define ECHO_PIN 7

// RGB
// #define RGB_B D5
// #define RGB_G 1 //D6 and D5 apparently dont exist? i have no idea how to fix this
// #define RGB_R D6


uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

#define SWITCH 10  // digital switch

#define MAX_DISTANCE 200  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
SparkFun_APDS9960 apds = SparkFun_APDS9960();

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ SENSOR1_PIN, SENSOR2_PIN, SENSOR3_PIN, SENSOR4_PIN, SENSOR5_PIN, SENSOR6_PIN, SENSOR7_PIN, SENSOR8_PIN }, SensorCount);

  //Setup RGB
  // pinMode(RGB_B, OUTPUT);
  // pinMode(RGB_G, OUTPUT);
  // pinMode(RGB_R, OUTPUT);
  apds.init();
  apds.enableLightSensor(false);

  //Switch
  pinMode(10, INPUT);

  //Setup Channel A
  pinMode(12, OUTPUT);  //Initiates Motor Channel A pin
  pinMode(9, OUTPUT);   //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT);  //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);   //Initiates Brake Channel A pin

  digitalWrite(8, HIGH);  //Engage the Brake for Channel A
  digitalWrite(9, HIGH);  //Engage the Brake for Channel B

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
  // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 13 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  Serial.println("Minimum Values for 8 sensors:");
  for (uint8_t i = 0; i < SensorCount; i++) {

    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println("Minimum Values for 8 sensors:");
  for (uint8_t i = 0; i < SensorCount; i++) {

    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void loop() {

  if (digitalRead(SWITCH) == 0) {
    digitalWrite(8, HIGH);  //Engage the Brake for Channel A
    digitalWrite(9, HIGH);  //Engage the Brake for Channel B
  } else {
    int distance = sonar.ping_cm();
    if (distance == 0) {
      digitalWrite(8, LOW);  //Disengage the Brake for Channel A
      digitalWrite(9, LOW);  //Disengage the Brake for Channel B

      // read calibrated sensor values and obtain a measure of the line position
      // from 0 to 8000 (for a white line, use readLineWhite() instead)
      uint16_t position = qtr.readLineBlack(sensorValues);
      //This method technically remembers when it last saw the black line, so the value stored in it will stay consistent
      //if we lose the black line

      for (uint8_t i = 0; i < SensorCount; i++) {
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

      distance = 45;

      //Motor A forward @ full speed
      digitalWrite(12, HIGH);           //Establishes forward direction of Channel A
      analogWrite(3, rightMotorSpeed);  //Spins the motor on Channel A at half speed

      //Motor B forward @ full speed
      digitalWrite(13, HIGH);           //Establishes forward direction of Channel B
      analogWrite(11, leftMotorSpeed);  //Spins the motor on Channel B at full speed
    }

    if (distance < 30) {
      digitalWrite(8, HIGH);  //Engage the Brake for Channel A
      digitalWrite(9, HIGH);  //Engage the Brake for Channel B

      if (!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light)) {
        Serial.println("Error reading light values");
      } else { //REPLACE WITH RGB LED DISPLAY INSTEAD(!)
        Serial.print("Ambient: ");
        Serial.print(ambient_light);
        Serial.print(" Red: ");
        Serial.print(red_light);
        Serial.print(" Green: ");
        Serial.print(green_light);
        Serial.print(" Blue: ");
        Serial.println(blue_light);
      }
    }
  }
}
