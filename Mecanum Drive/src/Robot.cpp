#include "WPILib.h"

//will test commit and push

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{

	   // Channels for the wheels
		CANJaguar frontLeft; // 5
		CANJaguar backLeft; // 4
		CANJaguar frontRight; // 2
		CANJaguar backRight; // 3
		Gyro gyro; // TBD -- MUST BE IN ANALOG INPUT 0 OR 1
		Victor Victor1;
		Victor Victor2;

	    //const static int joystickChannel	= 0;

		RobotDrive robotDrive;	// robot drive system
		Joystick stick; // only joystick
		Joystick stick2; // controller

	public:
		Robot() :
			//assign channels on the roboRio
				frontLeft(4),
				backLeft(5),
				frontRight(2),
				backRight(3),
				gyro(0),
				Victor1(1),
				Victor2(2),
				robotDrive(frontLeft, backLeft,
						   frontRight, backRight),	// these must be initialized in the same order
				stick(0), // as they are declared above.
				stick2(1)
		{
			robotDrive.SetExpiration(0.1);

			backLeft.SetSpeedMode(backLeft.QuadEncoder, 250, 0.4, 0.01, 0); //  (encoder, revCount,
			frontLeft.SetSpeedMode(frontLeft.QuadEncoder, 250, 0.4, 0.01, 0);//  P,I,D values)
			frontRight.SetSpeedMode(frontRight.QuadEncoder, 250, 0.4, 0.01, 0);
			backRight.SetSpeedMode(backRight.QuadEncoder, 250, 0.4, 0.01, 0);

			backLeft.EnableControl(); //Enable control of Jaguars
			frontLeft.EnableControl();
			frontRight.EnableControl();
			backRight.EnableControl();
		}

		/**
		 * Runs the motors with Mecanum drive.
		 */
		void OperatorControl()
		{
			robotDrive.SetSafetyEnabled(false);
			robotDrive.SetMaxOutput(250);
			gyro.Reset();

			float x;
			float y;
			float z;

			float gyroChangeRate;

			//-------------------------------------------------------------------//

			//Used to determine the gyro change rate prior to moving.
			if (true)
			{
			float gyroStart = gyro.GetAngle();
			Wait(1.0);
			float gyroEnd = gyro.GetAngle();
			gyroChangeRate = gyroEnd - gyroStart;

			SmartDashboard::PutNumber("Gyro Start", gyroStart);
			SmartDashboard::PutNumber("Gyro End", gyroEnd);
			}

			SmartDashboard::PutNumber("Gyro Rate", gyroChangeRate);

			bool noRotFlag = false;
			float consGyro;

			//-------------------------------------------------------------------//

			while (IsOperatorControl() && IsEnabled())
			{
	        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
	        	// This sample does not use field-oriented drive, so the gyro input is set to zero.

				x = -stick.GetX();
				y = -stick.GetTwist();
				z = -stick.GetY();

				if ((x < .15) && (x > -.15))
				{
					x = 0.0;
				}

				if ((y < .4) && (y > -.4))
				{
					y = 0.0;
				}

				if ((z < .15) && (z > -.15))
				{
					z = 0.0;
				}

				//---------------------------------------------------------------//

				if (y > .4)
				{
					y = y - .4;
				}

				else if (y < -.4)
				{
					y = y + .4;
				}

				//---------------------------------------------------------------//

				if (z != 0.0) // If the driver is rotating
				{
					//robotDrive.MecanumDrive_Cartesian(x, y, z); // Mecanum Drive using the x, y, z modified from the deadzone code
					noRotFlag = false;
				}

				else if ((z == 0) && ((x != 0.0) || (y != 0.0))) // If no rotation, and driver is
				{												 // moving on x or y axis
					if (noRotFlag == false)
					{
						noRotFlag = true;
						consGyro = gyro.GetAngle();
					}

					//robotDrive.MecanumDrive_Cartesian(x, y, (consGyro - gyro.GetAngle()));

					SmartDashboard::PutNumber("Gyro Test", (consGyro - gyro.GetAngle()));
				}

				else
				{
					//robotDrive.MecanumDrive_Cartesian(x, y, z);
				}

				//--------------------------------------------------------------//

				if(stick.GetRawButton(1))
				{
					x = x/2;
					y = y/2;
					z = z/2;
				}

				robotDrive.MecanumDrive_Cartesian(x, y, z);

				//-------------------------------------------------------------//

				SmartDashboard::PutNumber("Joystick X", x);
				SmartDashboard::PutNumber("Joystick Y", y);
				SmartDashboard::PutNumber("Joystick Z", z);
				SmartDashboard::PutNumber("Gyro Angle", -gyro.GetAngle());

				if(stick2.GetRawButton(5))
				{
					Victor1.Set(1.0);
				}
				else if (stick2.GetRawAxis(2) >= 0.5)
				{
					Victor1.Set(-1.0);
				}
				else
				{
					Victor1.Set(0);
				}

				if(stick2.GetRawButton(6))
				{
					Victor2.Set(1.0);
				}
				else if (stick2.GetRawAxis(3) >= 0.5)
				{
					Victor2.Set(-1.0);
				}
				else
				{
					Victor2.Set(0);
				}

				Wait(0.005); // wait 5ms to avoid hogging CPU cycles
			}
		}

	};

	START_ROBOT_CLASS(Robot);
