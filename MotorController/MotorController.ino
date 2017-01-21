#define DEBUG
#define SIMULATION

/*
The Coordinator port is the port by which the motor communicates to
the coordinator. The coordinator sends commands here, and the motor 
controller responds with errors or successes. 
*/
#define COORDINATOR_PORT Serial
enum MotorControls{RotateAbsolute = 0, MoveToPosition = 1, Stop = 2, Start = 3};
enum MotorErrors{MotionComplete = 0, RotationFailure = 1, MoveToPositionFailure = 2};

struct MotionValue
{
	MotorControls MotionType;
	int position;
};

// Global Motion Controls
boolean Armed = false;
boolean MotionQueued = false;
long position = 0; // Position is relative and resets after motion
long allowablePositionError = 3;
int rotation = 0; // Rotation is held globally and absolute.
int allowableRotationError = 3;
MotionValue curMotion;

int ARMEDLEDPIN = 13;

long updateInterval = 1000;
long lastUpdate = 0;

// Return Success = 0, Failure = 1
int ParseCommand(char ControlByte, String ControlArgument)
{
	ControlByte = ControlByte - '0';

	if(ControlByte == RotateAbsolute)
	{
		curMotion.MotionType = RotateAbsolute;
		curMotion.position = ControlArgument.toInt();
		MotionQueued = true;

#ifdef DEBUG
		Serial.print("Found rotate command with value of ");
		Serial.println(curMotion.position);
#endif
	}
	else if(ControlByte == MoveToPosition)
	{
		curMotion.MotionType = MoveToPosition;
		curMotion.position = ControlArgument.toInt();
		MotionQueued = true;

		#ifdef DEBUG
		Serial.print("Found Move command with value of ");
		Serial.println(curMotion.position);
		#endif
	}
	else if(ControlByte == Stop)
	{
		Armed = false;

		#ifdef DEBUG
		Serial.println("Disarmed");
		#endif
	}
	else if(ControlByte == Start)
	{
		Armed = true;

		#ifdef DEBUG
		Serial.println("Armed");
		#endif
	}
	else
	{
		#ifdef DEBUG
		Serial.println("Command Not Found");
		#endif

		return 1; // Command Not Found
	}

	return 0;
}

// Return Success = 0, Failure = 1
int CheckForCommands()
{
	enum ReadStates{ReadCommand, ReadComma, ReadArgument, end};

	char ControlByte;
	String ControlArgument;

	ReadStates state = ReadCommand;
	if(COORDINATOR_PORT.available())
	{
		while(COORDINATOR_PORT.available())
		{
			if(state == ReadCommand)
			{
				ControlByte = COORDINATOR_PORT.read();
				state = ReadComma;
			}
			else if(state == ReadComma)
			{
				char comma = COORDINATOR_PORT.read();

				if(comma != ',')
				{
					return 1;
				}

				state = ReadArgument;
			}
			else if(state == ReadArgument)
			{
				ControlArgument = COORDINATOR_PORT.readStringUntil('&');
				state = end;
			}
			else
			{
				break;
			}
		}

		return ParseCommand(ControlByte, ControlArgument);
	}

	return 0;
}

void StopAllMotors()
{

}

boolean IsRotationAcceptable()
{
	if(rotation > curMotion.position - allowableRotationError && rotation < curMotion.position + allowableRotationError)
	{
		return true;
	}

	return false;
}

int DoRotationMove()
{
	if(IsRotationAcceptable())
	{
		MotionQueued = false;

		#ifdef DEBUG
		Serial.println("Rotation complete!");
		#endif

		return 0;
	}
	else
	{
		#ifdef SIMULATION
		rotation = curMotion.position;
		#else

		#endif
	}

	return 0;
}

boolean IsPositionAcceptable()
{
	if(position > curMotion.position - allowablePositionError && position < curMotion.position + allowablePositionError)
	{
		return true;
	}

	return false;
}

int DoPositionMove()
{
	if(IsPositionAcceptable())
	{
		MotionQueued = false;

		#ifdef DEBUG
		Serial.println("Motion complete!");
		#endif

		return 0;
	}
	else
	{
		#ifdef SIMULATION
		position = curMotion.position;
		#else

		#endif
	}

	return 0;
}

int DetermineMotionTypeAndMove()
{
	if(curMotion.MotionType == RotateAbsolute)
	{
		return DoRotationMove();
	}
	else if(curMotion.MotionType == MoveToPosition)
	{
		return DoPositionMove();
	}

	return 0;
}

int DoMotion()
{
	if(Armed)
	{
		digitalWrite(ARMEDLEDPIN, HIGH);

		if(MotionQueued)
		{
			return DetermineMotionTypeAndMove();
		}
		else
		{
			return 0;
		}
	}
	else
	{
		digitalWrite(ARMEDLEDPIN, LOW);

		StopAllMotors();
		return 0;
	}

	return 0;
}

void setup()
{
	pinMode(ARMEDLEDPIN, OUTPUT);
	digitalWrite(ARMEDLEDPIN, LOW);

	COORDINATOR_PORT.begin(9600);
}

void loop()
{	
	CheckForCommands();
	int status = DoMotion();

	// A Status of 1 means in motion
	// A status greater than 1 means error
	// A status of 0 means ready for next command
	if(millis() - lastUpdate > updateInterval)
	{
		lastUpdate = millis();
		COORDINATOR_PORT.print(status);
		COORDINATOR_PORT.print('&');
	}
}