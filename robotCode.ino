#include <NewPing.h>
#include <SparkFun_APDS9960.h>
#include <Wire.h>

// the pins for each ir sensor
#define SENSOR1_PIN A3
#define SENSOR2_PIN A2
#define SENSOR3_PIN A1
#define SENSOR4_PIN A0
#define SENSOR5_PIN 5
#define SENSOR6_PIN 6

// Ultrasonic
#define TRIGGER_PIN 4
#define ECHO_PIN 7

#define BUZZER 10

//Sonar Distance
int distance = 0;

uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

#define SWITCH 1  // digital switch

#define MAX_DISTANCE 200  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.
SparkFun_APDS9960 apds = SparkFun_APDS9960();

const uint8_t SensorCount = 6;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

int lastSeenBlackLine = 0;
int currentPositionWeight = 0;

void setup() {

  Serial.begin(9600);
  Serial.println("Starting Setup: ");

  if (apds.init()) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  // Start running the APDS-9960 light sensor (no interrupts)
  if (apds.enableLightSensor(false)) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }

  //Switch
  pinMode(SWITCH, INPUT);

  digitalWrite(SWITCH, LOW);


  //
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(SENSOR3_PIN, INPUT);
  pinMode(SENSOR4_PIN, INPUT);
  pinMode(SENSOR5_PIN, INPUT);
  pinMode(SENSOR6_PIN, INPUT);



  //Setup Channel A
  pinMode(12, OUTPUT);  //Initiates Motor Channel A pin
  pinMode(9, OUTPUT);   //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT);  //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);   //Initiates Brake Channel A pin

  digitalWrite(8, HIGH);  //Engage the Brake for Channel A
  digitalWrite(9, HIGH);  //Engage the Brake for Channel B

  // Wait for initialization and calibration to finish
  delay(500);
}

void loop() {
  distance = sonar.ping_cm();
  delay(75);
  Serial.println(distance);
  delay(200);
  engageBrakes();
  if (digitalRead(SWITCH) == HIGH) {
    disengageBrakes();
    if (distance < 30 && distance != 0) {
      engageBrakes();

      Serial.println("The Robobitch is not moving");

      int r = red_light;
      int b = blue_light;
      int g = green_light;
      int a = ambient_light;

      Serial.println(r);
      Serial.println(b);
      Serial.println(g);
      Serial.println(a);

      if (!apds.readAmbientLight(ambient_light) || !apds.readRedLight(red_light) || !apds.readGreenLight(green_light) || !apds.readBlueLight(blue_light)) {
        Serial.println("Error reading light values");
      } else {

        if (a > 300) {
          Serial.print("We see white");
          white();
        }

        else if (a < 200 && r < 100 && b < 100 && g < 100) {
          Serial.print("We see black, Ambient Value - ");
          Serial.println(ambient_light);
          black();
        }

        else if (r > b && r > g) {
          Serial.print("We see red, Red Value - ");
          Serial.print(red_light);
          red();
        }

        else if (g > b && r < g) {
          Serial.print("We see green, Green Value - ");
          Serial.print(green_light);
          green();
        }

        else if (b > r && b > g) {
          Serial.print("We see blue, Blue Value - ");
          Serial.print(blue_light);
          blue();
        }

        delay(200);
      }
    }

    else if (distance == 0 || distance > 30) {
      int currentPosition = 0;

      printRawSensorValues();
      currentPositionWeight = calculatePositionWeight();
      currentPosition = map(currentPositionWeight, -6, 6, -120, 120);
      printCurrentSensorWeightAndRelativePosition(currentPosition);
      if (currentPositionWeight < -1) {
        rightMotorSpeed = 30 - abs(currentPosition) / 2;
        leftMotorSpeed = 30 + abs(currentPosition) / 2;
      } else if (currentPositionWeight > 1) {
        rightMotorSpeed = 30 + abs(currentPosition) / 2;
        leftMotorSpeed = 30 - abs(currentPosition) / 2;
      } else {
        rightMotorSpeed = 100;
        leftMotorSpeed = 100;
      }




      rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
      leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
      //Motor A forward @ full speed
      digitalWrite(12, HIGH);           //Establishes forward direction of Channel A
      analogWrite(3, rightMotorSpeed);  //Spins the motor on Channel A at current speed

      //Motor B forward @ full speed
      digitalWrite(13, LOW);            //Establishes forward direction of Channel B
      analogWrite(11, leftMotorSpeed);  //Spins the motor on Channel B at current speed
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

int calculatePositionWeight() {
  lastSeenBlackLine = currentPositionWeight;
  currentPositionWeight = ((-3) * (digitalRead(SENSOR1_PIN)) + (-2) * (digitalRead(SENSOR2_PIN)) + (-1) * (digitalRead(SENSOR3_PIN)) + (1) * (digitalRead(SENSOR4_PIN)) + (2) * (digitalRead(SENSOR5_PIN)) + (3) * (digitalRead(SENSOR6_PIN)));

  if ((digitalRead(SENSOR1_PIN)) == 1 && (digitalRead(SENSOR2_PIN)) == 1 && (digitalRead(SENSOR3_PIN)) == 1 && (digitalRead(SENSOR4_PIN)) == 1 && (digitalRead(SENSOR5_PIN)) == 1 && (digitalRead(SENSOR6_PIN)) == 1) {
    return lastSeenBlackLine;
  } else {
    return currentPositionWeight;
  }
}

//DEBUG ZONE


void printRawSensorValues() {
  // //Raw Values for testing

  Serial.println("Raw Values of sensors, in order:");

  Serial.print(digitalRead(SENSOR1_PIN));
  Serial.print('\t');

  Serial.print(digitalRead(SENSOR2_PIN));
  Serial.print('\t');

  Serial.print(digitalRead(SENSOR3_PIN));
  Serial.print('\t');

  Serial.print(digitalRead(SENSOR4_PIN));
  Serial.print('\t');

  Serial.print(digitalRead(SENSOR5_PIN));
  Serial.print('\t');

  Serial.print(digitalRead(SENSOR6_PIN));
  Serial.println('\t');
}

void printCurrentDistanceFromObsstacle() {
  Serial.print("Current distance to the closest obstacle received from Sonar: ");
  Serial.println(distance);
}

void printCurrentSensorWeightAndRelativePosition(int position) {
  Serial.print("Current Sensor Weight Is: ");
  Serial.println(currentPositionWeight);
  Serial.print("Last Seen Black Line on: ");
  if (lastSeenBlackLine > 0) {
    Serial.println("Right Side");
  }
  if (lastSeenBlackLine < 0) {
    Serial.println("Left Side");
  } else {
    Serial.println("In the middle");
  }
  Serial.print("Current Relative Position is: ");
  Serial.println(position);
}


void white() {
  for (int i = 0; i < 1; i++) {
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(700);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(700);
    noTone(BUZZER);
    delay(3000);
  }
}

void black() {
  for (int i = 0; i < 1; i++) {
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(2000);
  }
}

void green() {
  for (int i = 0; i < 1; i++) {
    tone(BUZZER, 450);
    delay(700);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(700);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(2000);
  }
}

void blue() {
  for (int i = 0; i < 1; i++) {
    tone(BUZZER, 450);
    delay(700);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(2000);
  }
}

void red() {
  for (int i = 0; i < 1; i++) {
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(700);
    noTone(BUZZER);
    delay(400);
    tone(BUZZER, 450);
    delay(200);
    noTone(BUZZER);
    delay(2000);
  }
}
