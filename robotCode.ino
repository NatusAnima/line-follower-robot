#include <NewPing.h>
#include <SparkFun_APDS9960.h>

// the pins for each ir sensor
#define SENSOR1_PIN A5
#define SENSOR2_PIN A4
#define SENSOR3_PIN A3
#define SENSOR4_PIN A2
#define SENSOR5_PIN A1
#define SENSOR6_PIN A0

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

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

int lastSeenBlackLine = 0;
int currentPositionWeight = 0;

const int sensorPins[] = { SENSOR1_PIN, SENSOR2_PIN, SENSOR3_PIN, SENSOR4_PIN, SENSOR5_PIN, SENSOR6_PIN };

void setup() {
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

  Serial.begin(9600);
}

void loop() {




  //Checking sensor values and print them to console
  for (int i = 0; i < SensorCount; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }

  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();





  if (digitalRead(SWITCH) == 0) {
    engageBrakes();
  } else {
    int distance = sonar.ping_cm();
    delay(75);
    if (distance == 0) {
      int currentPosition = 0;
      lastSeenBlackLine = currentPositionWeight;
      currentPositionWeight = calculatePositionWeight();
      currentPosition = map(currentPositionWeight, -6, 6, -255, 255);

      if (linePosition < 0){
        rightMotorSpeed = rightMotorSpeed + currentPosition;
        leftMotorSpeed = leftMotorSpeed - currentPosition;
      }
      if (linePosition > 0){
        rightMotorSpeed = rightMotorSpeed - currentPosition;
        leftMotorSpeed = leftMotorSpeed + currentPosition;
      }
      else{
        rightMotorSpeed = 255;
        leftMotorSpeed = 255;
      }

      rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
      leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
      //Motor A forward @ full speed
      digitalWrite(12, HIGH);           //Establishes forward direction of Channel A
      analogWrite(3, rightMotorSpeed);  //Spins the motor on Channel A at current speed

      //Motor B forward @ full speed
      digitalWrite(13, HIGH);           //Establishes forward direction of Channel B
      analogWrite(11, leftMotorSpeed);  //Spins the motor on Channel B at current speed
    }

    if (distance < 30) {
      engageBrakes();

      if (!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light)) {
        Serial.println("Error reading light values");
      } else {  //REPLACE WITH RGB LED DISPLAY INSTEAD(!)
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

void engageBrakes() {
  digitalWrite(8, HIGH);  //Engage the Brake for Channel A
  digitalWrite(9, HIGH);  //Engage the Brake for Channel B
}

void disengageBrakes() {
  digitalWrite(8, LOW);  //Disengage the Brake for Channel A
  digitalWrite(9, LOW);  //Disengage the Brake for Channel B
}

void calculatePositionWeight() {
  currentPositionWeight = ((-3) * (SENSOR1_PIN) + (-2) * (SENSOR2_PIN) + (-1) * (SENSOR3_PIN) + (1) * (SENSOR4_PIN) + (2) * (SENSOR5_PIN) + (3) * (SENSOR6_PIN));
  if (currentPositionWeight == 0) {
    return lastSeenBlackLine;
  } else {
    return currentPositionWeight;
  }
}
