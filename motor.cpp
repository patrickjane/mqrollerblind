// ************************************************************************************
// motor.cpp
// ************************************************************************************
// ************************************************************************************
// Arduino Motor Shield implementation
// DC motor + encoder
// ************************************************************************************
// Copyright (c) 2020 - Patrick Fial
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
    encInt(encoderInt), currentPos(0), targetSteps(0), calState(csOff), calibratedSteps(0),
    shouldStop(0), newCalibrationSteps(0)
{ 
  attachInterrupt(digitalPinToInterrupt(encInt), Motor::encoder, RISING, this);
  pinMode(dirPin, OUTPUT); //Initiates Motor Channel A pin
  pinMode(brkPin, OUTPUT); //Initiates Brake Channel A pin
}

// ************************************************************************************
// encoder (static)
// ************************************************************************************

void Motor::encoder(Motor* mtr)
{
  mtr->runEncoder();
}

// ************************************************************************************
// moveForward
// ************************************************************************************

void Motor::moveForward(long int steps)
{
  if (!steps)
    return;

  opsMode = omForward;
  targetSteps = steps;

  digitalWrite(dirPin, HIGH);  // Establishes forward direction of Channel A
  digitalWrite(brkPin, LOW);   // Disengage the Brake for Channel A
  analogWrite(spdPin, 255);    // Spins the motor on Channel A at full speed
}

// ************************************************************************************
// moveBackward
// ************************************************************************************

void Motor::moveBackward(long int steps)
{
  if (!steps)
    return;

  opsMode = omBackward;
  targetSteps = steps;

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
  targetSteps = 0;

  digitalWrite(brkPin, HIGH); //Engage the Brake for Channel A
  analogWrite(spdPin, 0);    // motor speed to zero
  delay(500);

  if (calState == csRewind)
  {
    currentPos = calibratedSteps;
    calState = csOff;
  }
}

// ************************************************************************************
// goto
// ************************************************************************************

void Motor::goTo(double targetStepsitionPerc)
{
  int target = targetStepsitionPerc <= 0.01 ? 0 : calibratedSteps * (targetStepsitionPerc/100);
  int relativeSteps = currentPos - target;

  if (relativeSteps > 0)
    return moveBackward(relativeSteps);

  return moveForward(abs(relativeSteps));
}

// ************************************************************************************
// runEncoder
// ************************************************************************************

void Motor::runEncoder()
{
  if (opsMode == omStandby)
    return;

  if (opsMode == omBackward)
    currentPos--;
  else
    currentPos++;

  if (opsMode == omStandby || calState == csRunning)
    return;

  targetSteps--;

  if (targetSteps <= 0)
  {
    *shouldStop = true;
    targetSteps = 0;
  }
}

// ************************************************************************************
// calibrate
// ************************************************************************************

void Motor::calibrate()
{
  if (calState == csOff)
  {
    Serial.println("Starting calibration ...");
    currentPos = 0;
    calibratedSteps = 0;
    calState = csRunning;

    moveBackward();  
  }
  else
  {
    Serial.println("Calibration done, going back ...");
    
    stop();

    calState = csRewind;
    calibratedSteps = abs(currentPos);
    currentPos = 0;
    *newCalibrationSteps = calibratedSteps;
    
    moveForward(calibratedSteps);
  }
}
