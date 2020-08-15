#include "Arduino.h"
#include "vacuum.h"
#include <Stepper.h>

#define  STEPSREV    4096    // 64(fullsteps) * 64 (reduction ratio)

#define  COIL_1X     6
#define  COIL_2X     8
#define  COIL_3X     7
#define  COIL_4X     9

#define  COIL_1Y     2
#define  COIL_2Y     4
#define  COIL_3Y     3
#define  COIL_4Y     5

#define leftSensor A5
#define rightSensor A4
#define middleSensor A1

Stepper rightMotor(STEPSREV, COIL_1X, COIL_2X, COIL_3X, COIL_4X);
Stepper leftMotor(STEPSREV, COIL_1Y, COIL_2Y, COIL_3Y, COIL_4Y);

void startWorking()
{
	if(analogRead(middleSensor)>50 && analogRead(rightSensor)>200 && analogRead(leftSensor)>200)
{
  moveForward();
}
else
{
  moveBackward();
  turnRandom();
}
}

void enableSensors()
{
  pinMode(middleSensor,INPUT);
  pinMode(rightSensor,INPUT);
  pinMode(leftSensor,INPUT);
}

void sensorsCheck()
{
	Serial.begin(115200);
	Serial.println(analogRead(middleSensor));
	Serial.println(analogRead(rightSensor));
	Serial.println(analogRead(leftSensor));
	delay(500);
}
  
void setSpeed(int rpm)
{
  rightMotor.setSpeed(rpm);  // set the X motor speed.
  leftMotor.setSpeed(rpm);  // set the Y motor speed.
}

void turnBrush(unsigned n) //600 disengaged //1100 engaged
{ 

  for(int i=0; i<100; i++) 
                           
   {
  digitalWrite(11,1);
  delayMicroseconds(n); //the largest value that will produce an accurate delay is 16383. For longer delays use delay()
  digitalWrite(11,0);
  delay(15); //delay 15k us
  delayMicroseconds(5000-n);

  }

}

//Check for inactivity and turn off the steppers coils to save battery.
void stopWorking(){
  
    digitalWrite(COIL_1X, 0);
    digitalWrite(COIL_2X, 0);
    digitalWrite(COIL_3X, 0);
    digitalWrite(COIL_4X, 0);

    digitalWrite(COIL_1Y, 0);
    digitalWrite(COIL_2Y, 0);
    digitalWrite(COIL_3Y, 0);
    digitalWrite(COIL_4Y, 0);
     
}

void moveForward()
{
  rightMotor.step(-1);
  leftMotor.step(-1);
}

void moveBackward()
{
  for(int i=0; i<900; i++)
  {
  rightMotor.step(1);
  leftMotor.step(1);
  }

}

void turnRight()
{

  moveBackward();
  
  for(int i=1; i<4201; i++)
  {
	   
			  if(analogRead(middleSensor)<50 || analogRead(rightSensor)<200 || analogRead(leftSensor)<200)
			{
			  i=4200;
			}
		 rightMotor.step(-1);

  }

  
}

void turnLeft()
{
  moveBackward();
  
  for(int i=1; i<4201; i++)
  {
	   
			  if(analogRead(middleSensor)<50 || analogRead(rightSensor)<200 || analogRead(leftSensor)<200)
			{
			  i=4200;
			}
		 leftMotor.step(-1);

  }

}

void turnRandom()
{
  int randomTurn = 0;
  randomTurn = random(0,2);
  if(randomTurn==1)
  {
    turnRight();
  }
  else
  {
    turnLeft();
  }
}
