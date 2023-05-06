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


#include <QTRSensors.h>

#define NUM_SENSORS   8  // number of sensors used

// create an instance of the QTRSensor object
QTRSensorsRC qtrA((char[]) {SENSOR1_PIN, SENSOR2_PIN, SENSOR3_PIN, SENSOR4_PIN, SENSOR5_PIN, SENSOR6_PIN, SENSOR7_PIN, SENSOR8_PIN}, 
                   NUM_SENSORS);
 
void setup()
{
  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin


  // then start calibration phase and move the sensors over both
  // reflectance extremes they will encounter in your application:
  for (int i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtrA.calibrate();
    delay(20);
  }
}

void loop()
{
  unsigned int sensors[3];
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 2000, with 1000 corresponding to the line
  // over the middle sensor.
  int position = qtrA.readLine(sensors);
//This method technically remembers when it last saw the black line, so the value stored in it will stay consistent
//if we lose the black line

 int leftMotorSpeed = 100;
  int rightMotorSpeed = 100;
  // if all eight sensors see very low reflectance, take some appropriate action for this 
  // situation.
  if (sensors[0] > 750 && sensors[1] > 750 && sensors[2] > 750 && sensors[3] > 750 && sensors[4] > 750 && sensors[5] > 750 && sensors[6] > 750 && sensors[7] > 750)
  {
    // keep pushing forward soldier.
    leftMotorSpeed = 100;
    rightMotorSpeed = 100;
  }
 
  // compute our "error" from the line position.  We will make it so that the error is
  // zero when the middle sensors are over the line, because this is our goal.  Error
  // will range from -3000 to +3000.  If we have sensor 0 on the far left and sensor 7 on
  // the far right,  a reading of -3000 means that we see the line on the very far left and a reading
  // of +3000 means we see the line on the very far right.
  // zero middle sensors are sensor 4 and sensor 5
  int error = position - 3000;
 
 //The error range and the motor speed values might need to be adjusted(!)
 if (error < -2500) {  // the line is on the far left
  leftMotorSpeed = 0;
  rightMotorSpeed = 255;
} else if (error < -2000) {  // the line is on the left
  leftMotorSpeed = 50;
  rightMotorSpeed = 255;
} else if (error < -1500) {  // the line is slightly on the left
  leftMotorSpeed = 100;
  rightMotorSpeed = 255;
} else if (error < -1000) {  // the line is slightly on the left
  leftMotorSpeed = 150;
  rightMotorSpeed = 255;
} else if (error < -500) {  // the line is on the left
  leftMotorSpeed = 200;
  rightMotorSpeed = 255;
} else if (error > 2500) {  // the line is on the far right
  leftMotorSpeed = 255;
  rightMotorSpeed = 0;
} else if (error > 2000) {  // the line is on the right
  leftMotorSpeed = 255;
  rightMotorSpeed = 50;
} else if (error > 1500) {  // the line is slightly on the right
  leftMotorSpeed = 255;
  rightMotorSpeed = 100;
} else if (error > 1000) {  // the line is slightly on the right
  leftMotorSpeed = 255;
  rightMotorSpeed = 150;
} else if (error > 500) {  // the line is on the right
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
