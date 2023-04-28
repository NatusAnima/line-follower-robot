#include <QTRSensors.h>

#define NUM_SENSORS   8  // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2  // emitter is controlled by digital pin 2

// the pins for each sensor
#define SENSOR1_PIN A0
#define SENSOR2_PIN A1
#define SENSOR3_PIN A2
#define SENSOR4_PIN A3
#define SENSOR5_PIN A4
#define SENSOR6_PIN A5
#define SENSOR7_PIN A6
#define SENSOR8_PIN A7

// create an instance of the QTRSensor object
QTRSensorsRC qtrrc((unsigned char[]) {SENSOR1_PIN, SENSOR2_PIN, SENSOR3_PIN, SENSOR4_PIN, SENSOR5_PIN, SENSOR6_PIN, SENSOR7_PIN, SENSOR8_PIN}, 
                   NUM_SENSORS, TIMEOUT, EMITTER_PIN);

void setup()
{
  // initialize the serial communication
  Serial.begin(9600);
  
  // calibrate the sensors
  qtrrc.calibrate();
  
  // set the motor pins as output
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop()
{
  // read the sensor values
  unsigned int sensorValues[NUM_SENSORS];
  qtrrc.read(sensorValues);
  
  // compute the line position
  unsigned int position = qtrrc.readLine(sensorValues);
  
  // print the sensor values and line position
  Serial.print("Sensor values:");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print("Line position:");
  Serial.print(position);
  Serial.println();
  
  // adjust the motors based on the line position
  if (position < 3000)
  {
    // turn left
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
  }
  else if (position > 4000)
  {
    // turn right
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
  }
  else
  {
    // go straight
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
  }
}
