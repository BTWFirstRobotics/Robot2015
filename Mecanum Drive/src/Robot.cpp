#include "WPILib.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{
	   // Channels for the wheels
	   // 0 is right flipper, 1 is left flipper
		CANJaguar frontLeft; // 5
		CANJaguar backLeft; // 4
		CANJaguar frontRight; // 2
		CANJaguar backRight; // 0
		Gyro gyro; // TBD -- MUST BE IN ANALOG INPUT 0 OR 1
		RobotDrive robotDrive;	// robot drive system
		Joystick moveStick; // only joystick
		Joystick xboxStick; // controller
		AxisCamera camera;
		AnalogInput gyroAI;
		Timer timer;
		CANJaguar elevator;
		CANJaguar jawWheels;
		Relay jaws;
		Relay rightFlipper;
		Relay leftFlipper;
		DigitalInput rightJawSwitch;
		DigitalInput leftJawSwitch;
		IMAQdxSession session;
		Image *frame;

	public:
		Robot():
			//assign channels on the roboRio
				frontLeft(8), //4
				backLeft(4),  //5
				frontRight(6),//2
				backRight(3), //3
				gyro(0),
				robotDrive(&frontLeft, &backLeft,
						   &backRight, &frontRight),	// these must be initialized in the same order
				moveStick(0),                           // as they are declared above.
				xboxStick(1),
				camera("10.12.9.11"), //camera set to IP address
				gyroAI(0),
				elevator(2),
				jawWheels(7),
				jaws(3, jaws.kBothDirections),
				rightFlipper(0, rightFlipper.kBothDirections),
				leftFlipper(1, leftFlipper.kBothDirections),
				rightJawSwitch(9),
				leftJawSwitch(0)
		{
			robotDrive.SetExpiration(0.1);

			frontLeft.SetSpeedMode(frontLeft.Encoder, 250, 0.4, 0.01, 0);//  P,I,D values)
			backLeft.SetSpeedMode(backLeft.QuadEncoder, 360, 0.4, 0.01, 0); //  (encoder, revCount,
			frontRight.SetSpeedMode(frontRight.QuadEncoder, 250, 0.4, 0.01, 0);
			backRight.SetSpeedMode(backRight.QuadEncoder, 250, 0.4, 0.01, 0);

//			frontLeft.SetVoltageMode(frontLeft.QuadEncoder, 250);//  P,I,D values)
//			backLeft.SetVoltageMode(backLeft.QuadEncoder, 360); //  (encoder, revCount,
//			frontRight.SetVoltageMode(frontRight.QuadEncoder, 250);
//			backRight.SetVoltageMode(backRight.QuadEncoder, 250);

			frontLeft.EnableControl();
			backLeft.EnableControl(); //Enable control of Jaguars
			frontRight.EnableControl();
			backRight.EnableControl();

			jawWheels.SetVoltageMode();
			jawWheels.EnableControl();

			gyroAI.SetAverageBits(2);
			gyroAI.SetOversampleBits(4);

			frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

			IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
			IMAQdxConfigureGrab(session);
		}

		/**
		 * Runs the motors with Mecanum drive.
		 */
		void OperatorControl()
		{
			timer.Reset();
			timer.Start();
			robotDrive.SetSafetyEnabled(false);
			robotDrive.SetMaxOutput(250);
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

			// int picCount = 0;

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

//			CameraServer::GetInstance()->SetQuality(50);
//			CameraServer::GetInstance()->StartAutomaticCapture("cam0");

			IMAQdxStartAcquisition(session);


			while (IsOperatorControl() && IsEnabled())
			{
				IMAQdxGrab(session, frame, true, NULL);
				CameraServer::GetInstance()->SetImage(frame);
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
				x = moveStick.GetX();
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
					elevator.Set(1.0);
					SmartDashboard::PutString("Spike Status:", "Forward");
				}
				else if((xboxStick.GetPOV() == 225) || (xboxStick.GetPOV() == 135) || (xboxStick.GetPOV() == 180))
				{
					elevator.Set(-1.0);
					SmartDashboard::PutString("Spike Status:", "Reverse");
				}
				else// if (xboxStick.GetPOV() == -1)
				{
					elevator.Set(0.0);
					SmartDashboard::PutString("Spike Status:", "None");
				}

				//---------------------------------------------------------------//

				// Controls the wheels on the jaws
				if(xboxStick.GetRawButton(5))
				{
					jawWheels.Set(10.0);
					SmartDashboard::PutString("Jaw Wheel Status:", "Pulling");
				}
				else if(xboxStick.GetRawButton(6))
				{
					jawWheels.Set(-10.0);
					SmartDashboard::PutString("Jaw Wheel Status:", "Pushing");
				}
				else
				{
					jawWheels.Set(0.0);
					SmartDashboard::PutString("Jaw Wheel Status:", "None");
				}

				//---------------------------------------------------------------//

				// Controls the jaws
				if(xboxStick.GetRawButton(2))
				{
					jaws.Set(jaws.kForward);
					SmartDashboard::PutString("Jaw Status:", "Grabbing");
				}
				else if(xboxStick.GetRawButton(3))
				{
					if((leftJawSwitch.Get() == false) && (rightJawSwitch.Get() == false))
					{
						jaws.Set(jaws.kReverse);
						SmartDashboard::PutString("Jaw Status:", "Releasing");
					}
					else
					{
						jaws.Set(jaws.kOff);
						SmartDashboard::PutString("Jaw Status:", "Limit Hit");
					}
				}
				else
				{
					jaws.Set(jaws.kOff);
					SmartDashboard::PutString("Jaw Status:", "None");
				}

				//---------------------------------------------------------------//

				//Controls the flippers
				if(xboxStick.GetRawButton(4))
				{
					rightFlipper.Set(rightFlipper.kForward);
					leftFlipper.Set(leftFlipper.kForward);
					SmartDashboard::PutString("Flipper Status:", "Flipping");
				}
				else if(xboxStick.GetRawButton(1))
				{
					rightFlipper.Set(rightFlipper.kReverse);
					leftFlipper.Set(leftFlipper.kReverse);
					SmartDashboard::PutString("Flipper Status:", "Downing");
				}
				else
				{
					rightFlipper.Set(rightFlipper.kOff);
					leftFlipper.Set(leftFlipper.kOff);
					SmartDashboard::PutString("Flipper Status:", "None");
				}

				//---------------------------------------------------------------//

				if(moveStick.GetRawButton(1))
				{
					x = x/2;
					y = y/2;
					z = z/2;
				}

				robotDrive.MecanumDrive_Cartesian(x, y, z);


				SmartDashboard::PutNumber("Front Left Value", frontLeft.Get());
				SmartDashboard::PutNumber("Front Right Value", frontRight.Get());
				SmartDashboard::PutNumber("Back Left Value", backLeft.Get());
				SmartDashboard::PutNumber("Back Right Value", backRight.Get());

				//-------------------------------------------------------------//

				SmartDashboard::PutNumber("Joystick X", x);
				SmartDashboard::PutNumber("Joystick Y", y);
				SmartDashboard::PutNumber("Joystick Z", z);
				SmartDashboard::PutNumber("Gyro Angle", -gyro.GetAngle() - (timer.Get()*gyroChangeRate));
				SmartDashboard::PutNumber("Gyro Voltage", gyroAI.GetVoltage());
				SmartDashboard::PutNumber("Gyro.GetRate", gyro.GetRate());

				Wait(0.005); // wait 5ms to avoid hogging CPU cycles
			}

			IMAQdxStopAcquisition(session);
		}

		void Autonomous()
		{
			Threshold yellowToteThresholdHSL(35, 70, 100, 255, 150, 250);
			//yay
			ColorImage* img;
			BinaryImage* modifiedImg;

			while (IsAutonomous() && IsEnabled())
			{

				if (camera.IsFreshImage())
				{
					img = new ColorImage(IMAQ_IMAGE_RGB);

					if(camera.GetImage(img) == 1)
					{
						if(img->GetHeight() != 0)
						{
							modifiedImg = img->ThresholdHSL(yellowToteThresholdHSL);
							modifiedImg = modifiedImg->ConvexHull(false);

							// Below is an ugly function call which returns the dimensions of
							// the largest yellow object
							if (modifiedImg->GetNumberParticles() > 0)
							{
								SmartDashboard::PutString("Largest Particle Dimensions", std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.width) + " x " + std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.height) + " pixels");
								SmartDashboard::PutString("Particle Location", std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.left) + ", " + std::to_string(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.top));
								SmartDashboard::PutNumber("Ratio of Width to Height", float(float(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.width)/float(modifiedImg->GetOrderedParticleAnalysisReports()->at(0).boundingRect.height)));
							}
						}
					}

					delete img;
				}

				Wait(0.05);
			}
		}
	};

	START_ROBOT_CLASS(Robot);
