// ************************************************************************************
// motor.cpp
// ************************************************************************************
// ************************************************************************************
// Arduino Motor Shield implementation
// DC motor + encoder
// ************************************************************************************

// ************************************************************************************
// Includes/Definitions
// ************************************************************************************

#include <Arduino.h>
#include "motor.h"

// ************************************************************************************
// class Motor
// ************************************************************************************
// ctor/dtor
// ************************************************************************************

Motor::Motor(int directionPin, int brakePin, int speedPin, int encoderInt)
  : dirPin(directionPin), brkPin(brakePin), spdPin(speedPin), opsMode(omStandby),
    encInt(encoderInt), currentPos(0), targetPos(0), calState(csOff), calibratedSteps(1000)
{ 
  attachInterrupt(digitalPinToInterrupt(encInt), Motor::encoder, RISING, this);
  pinMode(dirPin, OUTPUT); //Initiates Motor Channel A pin
  pinMode(brkPin, OUTPUT); //Initiates Brake Channel A pin
}

// ************************************************************************************
// moveForward
// ************************************************************************************

void Motor::moveForward(long int steps)
{
  if (!steps)
    return;

  opsMode = omRunning;
  targetPos = steps;

  digitalWrite(dirPin, HIGH);  // Establishes forward direction of Channel A
  digitalWrite(brkPin, LOW);   // Disengage the Brake for Channel A
  analogWrite(spdPin, 255);    // Spins the motor on Channel A at full speed
}

// ************************************************************************************
// moveBackward
// ************************************************************************************

void Motor::moveBackward(long int steps)
{
  if (!steps && currentPos == 0)
    return;

  opsMode = omRunning;
  targetPos = steps;

  digitalWrite(dirPin, LOW);   // Establishes forward direction of Channel A
  digitalWrite(brkPin, LOW);   // Disengage the Brake for Channel A
  analogWrite(spdPin, 255);    // Spins the motor on Channel A at full speed
}    

// ************************************************************************************
// stop
// ************************************************************************************

void Motor::stop()
{
  opsMode = omStandby;
  
  digitalWrite(brkPin, HIGH); //Eengage the Brake for Channel A
  delay(100);
}

// ************************************************************************************
// encoder
// ************************************************************************************

void Motor::encoder(Motor* mtr)
{
  mtr->runEncoder();
}

// ************************************************************************************
// runEncoder
// ************************************************************************************

void Motor::runEncoder()
{
  if (opsMode == omStandby)
  {
    return;
  }

  currentPos++;

  if (targetPos && currentPos >= targetPos)
  {
    stop();
    currentPos = targetPos;
    targetPos = 0;    
  }
}

// ************************************************************************************
// calibrate
// ************************************************************************************

void Motor::calibrate()
{
  if (calState == csOff)
  {
    calState = csRunning;
    moveForward();  
  }
  else
  {
    calState = csOff;
    stop();
    calibratedSteps = currentPos;
    moveBackward(calibratedSteps);
  }
}

// ************************************************************************************
// goto
// ************************************************************************************

void Motor::goTo(double targetPositionPerc)
{
  Serial.print("Going to position ");
  Serial.println(targetPositionPerc);

  int targetStep = targetPositionPerc <= 0.001 ? 0 : calibratedSteps * (targetPositionPerc/100);

  Serial.print("Going to step ");
  Serial.println(targetStep);

  if (!targetStep)
    return moveForward(currentPos);

  if (targetStep > currentPos)
    return moveBackward(targetStep);

  return moveForward(targetStep);
}
