// ************************************************************************************
// motor.h
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

enum CalibrationState
{
  csOff,
  csRunning,
  csRewind
};

enum OperationMode
{
  omStandby,
  omForward,
  omBackward
};

// ************************************************************************************
// class Motor
// ************************************************************************************

class Motor
{
  public:

    Motor(int directionPin, int brakePin, int speedPin, int encoderInt);

    void moveForward(long int steps = -1);
    void moveBackward(long int steps = -1);
    void stop();
    void runEncoder();

    long int getMotorPos() { return currentPos; }
    long int getCalibratedSteps() { return calibratedSteps; }
    int getMotorPosPerc() { return ((double)currentPos / (double)calibratedSteps * 100.0); }

    void calibrate();
    void goTo(double targetPositionPerc);

    void setFlags(long int* newCalStps, bool* shdStp) { shouldStop = shdStp; newCalibrationSteps = newCalStps; };
    void setCalibration(long int calSteps) { calibratedSteps = calSteps; }
    void setPosition(long int curPos) { currentPos = curPos; }

    bool isMoving() { return opsMode != omStandby; }
    bool isCalibrating() { return calState != csOff; }
    
  private:

    static void encoder(Motor* mtr);

    bool* shouldStop;
    long int* newCalibrationSteps;

    long int targetSteps;
    volatile long int currentPos;
    long int calibratedSteps;
    
    int dirPin, brkPin, spdPin, encInt;
    CalibrationState calState;
    OperationMode opsMode;
};
