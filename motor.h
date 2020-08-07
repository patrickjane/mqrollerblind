// ************************************************************************************
// motor.h
// ************************************************************************************
// ************************************************************************************
// Arduino Motor Shield implementation
// DC motor + encoder
// ************************************************************************************

// ************************************************************************************
// Includes/Definitions
// ************************************************************************************

enum CalibrationState
{
  csOff,
  csRunning
};

enum OperationMode
{
  omStandby,
  omRunning  
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

    void calibrate();
    void goTo(double targetPositionPerc);
    
  private:

    static void encoder(Motor* mtr);
  
    long int targetPos;
    volatile long int currentPos;
    long int calibratedSteps;
    
    int dirPin, brkPin, spdPin, encInt;   
    CalibrationState calState;
    OperationMode opsMode;
};
