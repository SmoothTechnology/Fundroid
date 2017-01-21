/*
Motor port controls the motors via UART. It sends commands 
the describe a rotation and position.
The motor controller returns a value for success or failure.
Upon failure, the motor controller tells the coordinator where
it thinks the motors went.
*/
#define MOTOR_PORT Serial1
enum MotorControls{RotateAbsolute = 0, MoveToPosition = 1, Stop = 2};
enum MotorErrors{MotionComplete = 0, RotationFailure = 1, MoveToPositionFailure = 2};

/* 
Arm port gives commands to the controlling Teensy based on
the actions that the arms can do.
It tells the arms to move up and down, and it tells the
hands to close and open. The arms return a success or failure flag
*/
#define ARM_PORT Serial2
enum ArmControls{RaiseArms = 0, LowerArms = 1, CloseHands = 2, OpenHands = 3};

void setup()
{
	MOTOR_PORT.begin(9600);
	ARM_PORT.begin(9600);
}

void loop()
{
	
}