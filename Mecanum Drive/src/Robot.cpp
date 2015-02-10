#include "WPILib.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{
	   // Channels for the wheels
		Victor frontLeft; // 50
		Talon backLeft; // 4
		Victor frontRight; // 2
		Talon backRight; // 3
		Gyro gyro; // TBD -- MUST BE IN ANALOG INPUT 0 OR 1
		Victor Victor1;
		Victor Victor2;
		RobotDrive robotDrive;	// robot drive system
		Joystick moveStick; // only joystick
		Joystick xboxStick; // controller
		AxisCamera camera;
		AnalogInput gyroAI;
		Timer timer;
		Relay elevator;
		Servo servo1;
		//Threshold yellowToteThresholdHSL;
		//Relay wheelGrabber;
//
	public:
		Robot():
			//assign channels on the roboRio
				frontLeft(9), //4
				backLeft(7),  //5
				frontRight(8),//2
				backRight(6), //3
				gyro(0),
				Victor1(1),
				Victor2(2),
				robotDrive(&frontLeft, &backLeft,
						   &frontRight, &backRight),	// these must be initialized in the same order
				//robotDrive(9, 7, 8, 6),
				moveStick(0), // as they are declared above.
				xboxStick(1),
				camera("10.12.9.11"), //camera set to IP address
				gyroAI(0),
				elevator(0, elevator.kBothDirections),
				servo1(2)
		{
			robotDrive.SetExpiration(0.1);

			//yellowToteThresholdHSL(35, 70, 100, 255, 150, 250);

			/*
			backLeft.SetSpeedMode(backLeft.QuadEncoder, 250, 0.4, 0.01, 0); //  (encoder, revCount,
			frontLeft.SetSpeedMode(frontLeft.QuadEncoder, 250, 0.4, 0.01, 0);//  P,I,D values)
			frontRight.SetSpeedMode(frontRight.QuadEncoder, 250, 0.4, 0.01, 0);
			backRight.SetSpeedMode(backRight.QuadEncoder, 250, 0.4, 0.01, 0);

			backLeft.EnableControl(); //Enable control of Jaguars
			frontLeft.EnableControl();
			frontRight.EnableControl();
			backRight.EnableControl();
			*/
			gyroAI.SetAverageBits(2);
			gyroAI.SetOversampleBits(4);
		}

		/**
		 * Runs the motors with Mecanum drive.
		 */
		void OperatorControl()
		{
			timer.Reset();
			timer.Start();
			robotDrive.SetSafetyEnabled(false);
			//robotDrive.SetMaxOutput(250);
			gyro.Reset();

			SmartDashboard::PutNumber("H_Value_Min", 35);
			SmartDashboard::PutNumber("H_Value_Max", 70);
			SmartDashboard::PutNumber("S_Value_Min", 100);
			SmartDashboard::PutNumber("S_Value_Max", 255);
			SmartDashboard::PutNumber("L_Value_Min", 150);
			SmartDashboard::PutNumber("L_Value_Max", 250);


			//Threshold yellowToteThresholdHSL(50, 70, 127, 255, 40, 225); //works kinda
			//Threshold yellowToteThresholdHSL(0, 70, 100, 255, 40, 240); //works well
			//Threshold yellowToteThresholdRGB(80, 255, 127, 255, 5, 150);

			Threshold yellowToteThresholdHSL(35, 70, 100, 255, 150, 250);

			float x;
			float y;
			float z;

			int picCount = 0;

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

			bool notRotating = false;
			float consGyro;

			camera.WriteRotation(camera.kRotation_180);

			//-------------------------------------------------------------------//

			while (IsOperatorControl() && IsEnabled())
			{
				//--------------------------------------------------------------------------------
				/*if(camera.IsFreshImage())
				{
					camera.GetImage(img);
					modifiedImg = img->ThresholdHSL(yellowToteThresholdHSL);
					//modifiedImg = modifiedImg->ConvexHull(false);

					//Below is an ugly function call which returns the width of the largest yellow object
					if (modifiedImg->GetNumberParticles() > 0)
					{
						SmartDashboard::PutString("Largest Particle Dimensions", std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.width) + " x " + std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.height) + " pixels");
					}
				}

				if(stick.GetRawButton(9) || stick.GetRawButton(10))
				{
					modifiedImg = img->ThresholdHSL(yellowToteThresholdHSL);
					modifiedImg = modifiedImg->ConvexHull(false);
					if (modifiedImg->GetNumberParticles() > 0)
					{
						SmartDashboard::PutString("Largest Particle Dimensions", std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.width) + " x " + std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.height) + " pixels");
					}
					yellowToteThresholdHSL = Threshold(int(SmartDashboard::GetNumber("H_Value_Min")),
											  int(SmartDashboard::GetNumber("H_Value_Max")),
											  int(SmartDashboard::GetNumber("S_Value_Min")),
											  int(SmartDashboard::GetNumber("S_Value_Max")),
											  int(SmartDashboard::GetNumber("L_Value_Min")),
											  int(SmartDashboard::GetNumber("L_Value_Max")));
					std::string fileString = "/Feb1_actualImage" + std::to_string(picCount) + ".bmp";

					//camera.GetImage(img); // set data received from camera to img look at nivision.h for image drawing functions
					img->Write(fileString.c_str());
					fileString = "/Feb1_processedImage" + std::to_string(picCount) + ".bmp";
					modifiedImg->Write((fileString.c_str()));

					picCount++;

					SmartDashboard::PutString("Picture: ", std::to_string(picCount));
				}*/

				//--------------------------------------------------------------------------------

	        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
	        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
				x = -moveStick.GetX();
				y = -moveStick.GetTwist();
				z = -moveStick.GetY();

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
					notRotating = false;
				}

				else if ((z == 0) && ((x != 0.0) || (y != 0.0))) // If no rotation, and driver is
				{												 // moving on x or y axis
					if (notRotating == false)
					{
						notRotating = true;
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

				// Controls the elevator
				if((xboxStick.GetPOV() == 45) || (xboxStick.GetPOV() == 315) || (xboxStick.GetPOV() == 0))
				{
					//relay.Set(relay.kOn);
					//elevator.Set(elevator.kForward);
					backRight.Set(1.0);
					SmartDashboard::PutString("Spike Status:", "Forward");
				}
				else if((xboxStick.GetPOV() == 225) || (xboxStick.GetPOV() == 135) || (xboxStick.GetPOV() == 180))
				{
					//relay.Set(relay.kOn);
					//elevator.Set(elevator.kReverse);
					backRight.Set(-1.0);
					SmartDashboard::PutString("Spike Status:", "Reverse");
				}
				else// if (xboxStick.GetPOV() == -1)
				{
					//elevator.Set(elevator.kOff);
					backRight.Set(0.0);
					SmartDashboard::PutString("Spike Status:", "None");
				}

				//---------------------------------------------------------------//

				// Controls the wheels on the jaws
				if(xboxStick.GetRawButton(5))
				{
					backLeft.Set(1.0);
				}
				else if(xboxStick.GetRawButton(6))
				{
					backLeft.Set(-1.0);
				}
				else
				{
					backLeft.Set(0.0);
				}

				//---------------------------------------------------------------//

				// Controls thhe jaws
				if(xboxStick.GetRawButton(2))
				{
					frontRight.Set(0.5);
				}
				else if(xboxStick.GetRawButton(3))
				{
					frontRight.Set(-0.5);
				}
				else
				{
					frontRight.Set(0.0);
				}

				//---------------------------------------------------------------//

				if(moveStick.GetRawButton(1))
				{
					x = x/2;
					y = y/2;
					z = z/2;
				}


				//UNCOMMENT AFTER GRABBING TESTING!!!!!!!!!!!!!!!!
				//robotDrive.MecanumDrive_Cartesian(x, y, z);


				SmartDashboard::PutNumber("Front Left Value", frontLeft.Get());
				SmartDashboard::PutNumber("Front Right Value", frontRight.Get());
				SmartDashboard::PutNumber("Back Left Value", backLeft.Get());
				SmartDashboard::PutNumber("Back Right Value", backRight.Get());

				if(xboxStick.GetRawButton(7))
				{
					servo1.SetAngle(1.0);
				}
				else if(xboxStick.GetRawButton(8))
				{
					servo1.SetAngle(0.0);
				}
				else
				{
					servo1.SetAngle(.5);
				}

				//-------------------------------------------------------------//

				SmartDashboard::PutNumber("Joystick X", x);
				SmartDashboard::PutNumber("Joystick Y", y);
				SmartDashboard::PutNumber("Joystick Z", z);
				SmartDashboard::PutNumber("Gyro Angle", -gyro.GetAngle() - (timer.Get()*gyroChangeRate));
				SmartDashboard::PutNumber("Gyro Voltage", gyroAI.GetVoltage());
				SmartDashboard::PutNumber("Gyro.GetRate", gyro.GetRate());

				/*if(stick2.GetRawButton(6))
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
				}*/

				Wait(0.005); // wait 5ms to avoid hogging CPU cycles
			}
		}

		void Autonomous()
		{
			Threshold yellowToteThresholdHSL(35, 70, 100, 255, 150, 250);

			ColorImage* img;
			BinaryImage* modifiedImg;

			while (IsAutonomous() && IsEnabled())
			{
				img = new ColorImage(IMAQ_IMAGE_RGB);

				if(camera.GetImage(img) == 1)
				{
					if(img->GetHeight() != 0)
					{
						modifiedImg = img->ThresholdHSL(yellowToteThresholdHSL);
						modifiedImg = modifiedImg->ConvexHull(false);

						//Below is an ugly function call which returns the width of the largest yellow object
						if (modifiedImg->GetNumberParticles() > 0)
						{
							SmartDashboard::PutString("Largest Particle Dimensions", std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.width) + " x " + std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.height) + " pixels");
						}
					}
				}

				delete img;

				Wait(0.05);
			}
		}

	};

	START_ROBOT_CLASS(Robot);
