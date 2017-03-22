#include "WPILib.h"
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <WPILib.h>
#include <Timer.h>
#include <AHRS.h>
#include <CANSpeedController.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include "CANTalon.h"
#include <Math.h>

#define POT_MAX_VALUE 40
#define POT_MIN_VALUE 10
#define ANGLE 45
#define WIGGLEROOM .5
#define MXP_IO_VOLTAGE (double)3.3f /* Alternately, 5.0f   */
#define MIN_AN_TRIGGER_VOLTAGE (double)0.76f
#define MAX_AN_TRIGGER_VOLTAGE MXP_IO_VOLTAGE - (double)2.0f
#define AUTO_ONE 100
#define AUTO_TWO 100
#define AUTO_THREE 100
#define AUTO_FOUR 100
#define AUTO_FIVE 100

#define I2C_SLAVE_ADR 0x64

static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER = 9;
static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER = 3;
static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER = 1;
static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS = 10;
static const int NUM_ROBORIO_ONBOARD_PWM_PINS = 10;
static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS = 4;

typedef unsigned char byte;

enum PinType
{
	DigitalIO, PWMs, AnalogIn, AnalogOut
};
int GetChannelFromPin(PinType type,int io_pin_number)
{
	int roborio_channel = 0;
	if(io_pin_number < 0)
	{
		throw std::runtime_error("Error:  navX MXP I/O Pin #");
	}
	switch(type)
	{
		case DigitalIO:
			if(io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER)
			{
				throw std::runtime_error("Error:  Invalid navX MXP Digital I/O Pin #");
			}
			roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS + (
					io_pin_number > 3 ? 4 : 0);
			break;
		case PWMs:
			if(io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER)
			{
				throw std::runtime_error("Error:  Invalid navX MXP Digital I/O Pin #");
			}
			roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
			break;
		case AnalogIn:
			if(io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER)
			{
				throw new std::runtime_error("Error:  Invalid navX MXP Analog Input Pin #");
			}
			roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
			break;
		case AnalogOut:
			if(io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER)
			{
				throw new std::runtime_error("Error:  Invalid navX MXP Analog Output Pin #");
			}
			roborio_channel = io_pin_number;
			break;
	}
	return roborio_channel;
}

//When deploying Code: Turn off Wifi

//If there is an Error With Microsoft Visual Studios: Go to Project->Properties->
//C/C++ General-> Preprocessors Include Paths...->Go the the Providers Tab->
//Hit Apply-> Uncheck CDT Cross GCC Built-in Compiler Settings-> Hit Apply
//Reckeck CDT Cross GCC Built-in Complier Settings-> Hit Apply-> close and build
class Robot:public frc::IterativeRobot
{

	public:

	Robot() :
			rr(1), rf(2), lr(4), lf(5), outakeEncoder(3)
	{

		i2cthing = new I2C(I2C::kOnboard, I2C_SLAVE_ADR);
		//Generic initialization code
		m_robotDrive.SetExpiration(0.1);
		//Inverts the left side
		m_robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		m_robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		timer.Start();  //Initializes the timer
		autocounter = 0;

		//This try catch block checks to see if an error instantiating the navx is thrown
		try
		{
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
			ahrs = new AHRS(SPI::Port::kMXP);
		}
		catch(std::exception& ex)
		{
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if(ahrs)
		{
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		}

	}

	private:

	frc::Joystick stick
	{ 0 };         // Only joystick
	frc::XboxController xbox
	{ 1 };

	double gyroHeading = 0;bool gyroLatch = false;
	float kgyroManip = 1;

	AHRS *ahrs;
	// 0  2
	// 1  3

	//Ultrasonic Variables
	frc::Ultrasonic US
	{ 0, 1 };
	frc::DigitalOutput gearUSTrig
	{ 0 };
	frc::DigitalInput gearUSEcho
	{ 1 };
	float gearUSTimeStart = 0;bool gearTimeReset = 0;
	float gearTimeDelay = 0;

	/*frc::AnalogInput ultrasonicTest { 0 };
	 const int Vcc = 5;
	 const double Vm = null; */

	//double Vi = Vcc/512;
	//TPixy<LinkI2C> Pixy;
	float Pixyx1 = 0;
	float Pixyy1 = 0;
	float Pixyh1 = 0;
	float Pixyw1 = 0;
	float Pixyx2 = 0;
	float Pixyy2 = 0;
	float Pixyh2 = 0;
	float Pixyw2 = 0;
	int Framecount = 0;
	int framelimit[15];
	float FocalLength = 0.110236;
	int screenwidth = 640;
	int screenheight = 400;

	//Outputs of Collin Pixy Code
	float StrafePct = 0;
	float FrdRevPct = 0;
	float TurntoAngle = 0;

	//Allowed Range
	int XRange = 5;
	int YRange = 5;
	int HRange = 5;
	int WRange = 5;

	//Testing Values
	//Almost Done

	//Right in front of all need to do is drive straight
	float Pixyx1Almost = 215; //Used
	float Pixyy1Almost = 95;
	float Pixyh1Almost = 51;
	float Pixyw1Almost = 30; //Used
	float Pixyx2Almost = 302; //Used
	float Pixyy2Almost = 96;
	float Pixyh2Almost = 48;
	float Pixyw2Almost = 19;

	//Almost Done Midpoint
	float XAlmostmid = 0;
	float YAlmostmid = 0;

	//Drive Control
	float CameraTurnTo = 0;
	float CameraDriveFrdRev = 0;
	float CameraStrafe = 0;

	//Peg Mid Point
	float Xmid = 0;
	float Ymid = 0;

	//Allow Next Steps
	bool PixyNoTargets = true;bool AllowStrafe = false;bool AllowNormalDrive =
	false;bool Target1Good = false;bool Target2Good = false;bool GearPlaced =
	false;

	I2C *i2cthing;

	CANTalon rr; /*left front */
	CANTalon rf;/*left rear */
	CANTalon lr; /*right front */
	CANTalon lf; /*right rear */
	CANTalon outakeEncoder; /*outake Encoder */

	frc::RobotDrive m_robotDrive
	{ rr, rf, lr, lf };
	frc::TalonSRX groundIntakeMotor
	{ 2 }; //Motor for the ground intake
	frc::TalonSRX outakeMotor
	{ 3 }; //Motor for the cloth lifting thing
	frc::TalonSRX climberMotor
	{ 4 }; //Motor for climbing
	frc::Servo gearServoLeft
	{ 0 };
	frc::Servo gearServoRight
	{ 1 };

	//Values for determining a deadband for control
	float joystickDeadBandX = 0;
	float joystickDeadBandY = 0;
	float joystickDeadBandZ = 0;
	float driveMax = 0; //Variable to limit motors to the same Max
	//Variables for motor values
	float frontLeft = 0;
	float frontRight = 0;
	float rearLeft = 0;
	float rearRight = 0;

	//Drive Latch Variables
	bool driveToggle = false;
	float driveGyro = 0;bool driveLatch = false;

	//GO Latch Variables
	bool gearToggle = false;bool gearLatch = false;
	//bool gearServoLatch = false;

	//Temporary Value(s)
	float tempA = 0;

	int lockHead = 0;

	float autoTime = 0;bool setTime = false;
	int autonum = 0;

	float gearPegAngle = 60;
	int gearAngL = gearPegAngle;
	int gearAngC = 0;
	int gearAngR = -gearPegAngle;
	int targeting_step = 0;
	float Obj1[4] =
	{ 0, 0, 0, 0 };
	float Obj2[4] =
	{ 0, 0, 0, 0 };
	float LeftObj[4] =
	{ 0, 0, 0, 0 };
	int xMax = 319;
	int xMin = 0;
	int gearDirection = 0;
	float holding_angle = 0;
	float kgearLining = 0.25;
	int checkStep = 0;
	//Find these values
	int targetWidth = 40;
	int xLock = 220;

	bool targetHoldingAngle = false;
	int targetAngel = 0;

	int autocounter;

	frc::Timer timer;
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();

	std::unique_ptr<frc::Command> autocommand;
	frc::SendableChooser<frc::Command*> chooser;

	void RobotInit() override
	{

		//chooser.AddDefault("Do Nothing", new Pickup());
		frc::SmartDashboard::PutNumber("Autonomous", 0);
	}

	void AutonomousInit() override
	{
		timer.Reset();
		timer.Start();

		ahrs->Reset();

		autonum = frc::SmartDashboard::GetNumber("Autonomous", 0);
		frc::SmartDashboard::PutString("Feedback: ", "Starting Auto...");

	}

	void AutonomousPeriodic() override
	{
		gearServoLeft.SetAngle(90);
		gearServoRight.SetAngle(0);
		switch(autonum)
		{
			case 0:
				m_robotDrive.MecanumDrive_Cartesian(0, 0, (((ahrs->GetAngle() + 0) / 180) * 4), 0);
				break;
			case 1:
				if(timer.Get() < 2)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, .5, (((ahrs->GetAngle() + 0) / 180) * 4), 0);
				}
				else
				{
					autonum = 0;
				}
				break;
			case 2:
				//Right Peg
				///Boosted timebecause lost sight of target
				if(timer.Get() < 1.7)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, .5, (((ahrs->GetAngle() + 0) / 180) * 4), 0);
				}
				else if((timer.Get() > 1.7) && !(ahrs->GetAngle() > -65) && !(ahrs->GetAngle() < -55))
				{
					m_robotDrive.MecanumDrive_Cartesian(0, 0, (((ahrs->GetAngle() + 60) / 180) * 4), 0);
					frc::SmartDashboard::PutNumber("AutoAngle", ahrs->GetAngle());
				}
				if((ahrs->GetAngle() > -65) && (ahrs->GetAngle() < -55))
				{
					PixyFunct();
					PixyDriveTakeover();

					frc::SmartDashboard::PutNumber("AutoStraf", StrafePct);
					frc::SmartDashboard::PutNumber("AutoFrd", FrdRevPct);

					if(timer.Get() > 4)
					{
						if(!setTime)
						{
							setTime = true;
							autoTime = timer.Get();
						}
						if((timer.Get() - autoTime) < 2)
						{
							m_robotDrive.MecanumDrive_Cartesian(0, .25, (((ahrs->GetAngle() + 60) / 180) * kgyroManip), 0);
						}
						else
						{
							autonum = 0;

						}
					}
				}

				break;
			case 3:
				//Left Peg
				///Boosted timebecause lost sight of target
				if(timer.Get() < 1.7)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, .5, (((ahrs->GetAngle() + 0) / 180) * 4), 0);
				}
				else if((timer.Get() > 1.7) && !(ahrs->GetAngle() < 65) && !(ahrs->GetAngle() > 55))
				{
					m_robotDrive.MecanumDrive_Cartesian(0, 0, (((ahrs->GetAngle() - 60) / 180) * 4), 0);
					frc::SmartDashboard::PutNumber("AutoAngle", ahrs->GetAngle());
				}
				if((ahrs->GetAngle() < 65) && (ahrs->GetAngle() > 55))
				{
					PixyFunct();
					PixyDriveTakeover();

					frc::SmartDashboard::PutNumber("AutoStraf", StrafePct);
					frc::SmartDashboard::PutNumber("AutoFrd", FrdRevPct);

					if(timer.Get() > 4)
					{
						if(!setTime)
						{
							setTime = true;
							autoTime = timer.Get();
						}
						if((timer.Get() - autoTime) < 2)
						{
							m_robotDrive.MecanumDrive_Cartesian(0, .25, (((ahrs->GetAngle() - 60) / 180) * kgyroManip), 0);
						}
						else
						{
							autonum = 0;

						}
					}
				}

				break;
			case 4:

				if(timer.Get() < 1)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, .5, 0, 0);
				}
				else
				{
					PixyFunct();
					PixyDriveTakeover();
				}
				if(timer.Get() > 4)
				{
					if(!setTime)
					{
						setTime = true;
						autoTime = timer.Get();
					}
					if((timer.Get() - autoTime) < 2)
					{
						m_robotDrive.MecanumDrive_Cartesian(0, .25, (((ahrs->GetAngle() - 0) / 180) * kgyroManip), 0);
					}
					else
					{
						autonum = 0;

					}
				}
				break;
			case 5:
				gearServoLeft.SetAngle(90);
				gearServoRight.SetAngle(0);
				if(timer.Get() < 1.7)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, .5, (((ahrs->GetAngle() - 0) / 180) * 2), 0);
				}
				else if((timer.Get() > 1.7) && (timer.Get() < 6))
				{
					m_robotDrive.MecanumDrive_Cartesian(0, .125, (((ahrs->GetAngle() - 0) / 180) * 2), 0);
				}
				else
				{
					m_robotDrive.MecanumDrive_Cartesian(0, 0, (((ahrs->GetAngle() - 0) / 180) * 2), 0);
				}
				break;
		}
	}
	;

	void PixyFunct()
	{
		byte buff[31];
		int translate[15];

		i2cthing->Read(0x64, 31, buff);

		if(!(buff[1] == buff[2]))
		{
			if(buff[0] == 0)
			{
				for(int i = 0;i < 30;i++)
				{
					buff[i] = buff[i + 1];
				}
			}
		}
		for(int i = 0;i < 15;i++)
		{
			translate[i] = buff[(2 * i) + 1] * 256 + buff[2 * i];
		}
		if(translate[0] == 43605 && translate[1] == 43605 && translate[8] == 43605)
		{
			Pixyx1 = translate[4];
			Pixyy1 = translate[5];
			Pixyw1 = translate[6];
			Pixyh1 = translate[7];

			Pixyx2 = translate[11];
			Pixyy2 = translate[12];
			Pixyw2 = translate[13];
			Pixyh2 = translate[14];
			PixyNoTargets = false;
			SmartDashboard::PutBoolean("PixyNoTargets", PixyNoTargets);
		}
		else if(translate[0] == 0 && translate[1] == 0 && translate[8] == 0)
		{
			//Target Non Existant
			PixyNoTargets = true;
			SmartDashboard::PutBoolean("PixyNoTargets", PixyNoTargets);
		}

		SmartDashboard::PutNumber("Pixyx1", Pixyx1);
		SmartDashboard::PutNumber("Pixyy1", Pixyy1);
		SmartDashboard::PutNumber("Pixyw1", Pixyw1);
		SmartDashboard::PutNumber("Pixyh1", Pixyh1);
		SmartDashboard::PutNumber("Pixyx2", Pixyx2);
		SmartDashboard::PutNumber("Pixyy2", Pixyy2);
		SmartDashboard::PutNumber("Pixyw2", Pixyw2);
		SmartDashboard::PutNumber("Pixyh2", Pixyh2);
		frc::SmartDashboard::PutNumber("Make Sure Updating", 676);

	}
	;

	float targetAdjustment(float currentAng,float targetAng)
	{
		frc::SmartDashboard::PutNumber("Fused Heading", ahrs->GetFusedHeading());
		float kAngAdjust = 2;

		float radCurrAng = (currentAng * 3.141) / 180;
		float radTarAng = (targetAng * 3.141) / 180;

		float cosCurAng = cos(radCurrAng);
		float cosTarAng = cos(radTarAng);
		float sinCurAng = sin(radCurrAng);
		float sinTarAng = sin(radTarAng);

		if((currentAng > 0) && (targetAng > 0) && (currentAng < 180) && (targetAng < 180))
		{
			frc::SmartDashboard::PutString("Target Feed: ", "Area 1");
			return (cosCurAng - cosTarAng) * kAngAdjust;
		}
		else if((currentAng > 180) && (targetAng > 180) && (currentAng < 360) && (targetAng < 360))
		{
			frc::SmartDashboard::PutString("Target Feed: ", "Area 2");
			return (cosTarAng - cosCurAng) * kAngAdjust;
		}
		else if((currentAng > 90) && (targetAng > 90) && (currentAng < 270) && (targetAng < 270))
		{
			frc::SmartDashboard::PutString("Target Feed: ", "Area 3");
			return (sinCurAng - sinTarAng) * kAngAdjust;
		}
		else if(((currentAng - 360) > -90) && ((targetAng - 360) > -90) && (currentAng < 90) && (targetAng < 90))
		{
			frc::SmartDashboard::PutString("Target Feed: ", "Area 4");
			return (sinTarAng - sinCurAng) * kAngAdjust;
		}
		else
		{
			int angDiff = currentAng - targetAng;
			int angSig = (angDiff / fabs(angDiff));

			if(angSig == 1)
			{
				if(angDiff > 180)
				{
					frc::SmartDashboard::PutString("Target Feed: ", "Area 5");
					return .25;
				}
				else if(angDiff < 180)
				{
					frc::SmartDashboard::PutString("Target Feed: ", "Area 6");
					return -.25;
				}
				else
				{
					frc::SmartDashboard::PutString("Target Feed: ", "Area 7");
					return 0;
				}
			}
			else if(angSig == 1)
			{
				if(fabs(angDiff) > 180)
				{
					return -.25;
				}
				else if(fabs(angDiff) < 180)
				{
					return .25;
				}
				else
				{
					return 0;
				}
			}
			else
			{
				return -0.05;
			}
		}
	}
	;

	void PixyDriveTakeover()
	{
		Xmid = ((Pixyx1 + Pixyx2) / 2);
		XAlmostmid = ((Pixyx1Almost + Pixyx2Almost) / 2) - 15;
		StrafePct = (XAlmostmid - Xmid) / 100;
		//StrafePct = StrafePct/fabs(StrafePct);
		//StrafePct = StrafePct *.5;

		FrdRevPct = (Pixyw1Almost - Pixyw1) / 70;
		//FrdRevPct = FrdRevPct/fabs(FrdRevPct);
		//FrdRevPct = FrdRevPct *.125;
		if(PixyNoTargets)
		{
			StrafePct = 0;
			FrdRevPct = 0;
			frc::SmartDashboard::PutString("Feedback: ", "Lost");
		}

		frc::SmartDashboard::PutNumber("Fused Heading", ahrs->GetFusedHeading());

		if(targetHoldingAngle == false)
		{

			if((sin(ahrs->GetFusedHeading() * (3.141 / 180)) > sin(-30 * (3.141 / 180))) && (sin(ahrs->GetFusedHeading() * (3.141 / 180)) < sin(30 * (3.141 / 180))))
			{
				targetAngel = 1;
			}
			else if(ahrs->GetFusedHeading() > 270 && ahrs->GetFusedHeading() < 330)
			{
				targetAngel = 2;
			}
			else if(ahrs->GetFusedHeading() > 30 && ahrs->GetFusedHeading() < 90)
			{
				targetAngel = 3;
			}
			else
			{
				targetAngel = 0;
			}

			targetHoldingAngle = true;
		}

		float testAng = 0;

		if(targetAngel == 1)
		{
			frc::SmartDashboard::PutString("Feedback: ", "Targeting Mid");
			testAng = (sin(0 * (3.141 / 180)) - sin(ahrs->GetFusedHeading() * (3.141 / 180))) * 2.1;
			frc::SmartDashboard::PutNumber("testAng", testAng);
		}
		else if(targetAngel == 2)
		{
			frc::SmartDashboard::PutString("Feedback: ", "Targeting -60");
			testAng = ((300 - ahrs->GetFusedHeading()) / 180) * 4;
			frc::SmartDashboard::PutNumber("testAng", testAng);
		}
		else if(targetAngel == 3)
		{
			frc::SmartDashboard::PutString("Feedback: ", "Targeting 60");
			testAng = ((60 - ahrs->GetFusedHeading()) / 180) * 4;
			frc::SmartDashboard::PutNumber("testAng", testAng);
		}
		else
		{
			frc::SmartDashboard::PutString("Feedback: ", "Targeting NULL");
			testAng = 0;
		}

		frc::SmartDashboard::PutNumber("StrafePct", StrafePct);
		frc::SmartDashboard::PutNumber("FrdRevPct", FrdRevPct);

		//m_robotDrive.MecanumDrive_Cartesian(StrafePct, FrdRevPct, (((heldAng - currentAngle) / 180) * 4), 0);
		m_robotDrive.MecanumDrive_Cartesian(StrafePct, FrdRevPct, -testAng, 0);
	}
	;

	void gearPegAngleTarget(float currentPeg,float xspeed,float yspeed)
	{
		holding_angle = currentPeg;
		frc::SmartDashboard::PutNumber("XSPEED", xspeed);
		frc::SmartDashboard::PutNumber("YSPEED", yspeed);
		frc::SmartDashboard::PutNumber("ANGLE", currentPeg);
		m_robotDrive.MecanumDrive_Cartesian(xspeed, yspeed, (((ahrs->GetAngle() - currentPeg) / 180) * kgyroManip), 0);
	}
	;

	int findTheLeft(int Obj1x,int Obj2x)
	{
		LeftObj[0] = 0;
		LeftObj[1] = 0;
		LeftObj[2] = 0;
		LeftObj[3] = 0;
		if(Obj2x == 0)
		{
			LeftObj[0] = Obj1[0];
			LeftObj[1] = Obj1[1];
			LeftObj[2] = Obj1[2];
			LeftObj[3] = Obj1[3];
			return Obj1x;
		}
		else if(Obj1x < Obj2x)
		{
			LeftObj[0] = Obj1[0];
			LeftObj[1] = Obj1[1];
			LeftObj[2] = Obj1[2];
			LeftObj[3] = Obj1[3];
			return Obj1x;
		}
		else if(Obj2x < Obj1x)
		{
			LeftObj[0] = Obj2[0];
			LeftObj[1] = Obj2[1];
			LeftObj[2] = Obj2[2];
			LeftObj[3] = Obj2[3];
			return Obj2x;
		}
		else
		{
			targeting_step = 9;
			return 0;
		}
	}
	;

	void PutGearOnPeg()
	{
		switch(targeting_step)
		{
			case 0:
				SmartDashboard::PutString("Feedback: ", "Align to angle of peg");
				checkStep = 1;
				if((ahrs->GetAngle() > gearAngL - 29) && (ahrs->GetAngle() < gearAngL + 29))
				{
					//Run Target Code for AngleL
					gearPegAngleTarget(gearAngL, 0, 0);
				}

				else if((ahrs->GetAngle() > gearAngC - 29) && (ahrs->GetAngle() < gearAngC + 29))
				{
					//Run Target Code for AngleC
					gearPegAngleTarget(gearAngC, 0, 0);
				}

				else if((ahrs->GetAngle() > gearAngR - 29) && (ahrs->GetAngle() < gearAngR + 29))
				{
					//Run Target Code for AngleR
					gearPegAngleTarget(gearAngR, 0, 0);
				}
				if((fabs(ahrs->GetAngle() - holding_angle)) < 5)
				{
					targeting_step = 7;
				}
				break;
			case 1:
				SmartDashboard::PutString("Feedback: ", "Determine if one strip is too far left or right");
				if(Obj2[0] == 0)
				{
					if(Obj1[0] <= (xMax / 2))
					{
						gearDirection = 1;
					}
					else if(Obj1[0] > (xMax / 2))
					{
						gearDirection = 2;
					}
					targeting_step = 3;
				}
				else
				{
					targeting_step = 2;
				}
				break;
			case 2:
				SmartDashboard::PutString("Feedback: ", "Determine if 2 strips are too far left or Right");
				if(Obj2[0] < Obj1[0])
				{
					gearDirection = 1;
					targeting_step = 3;
				}
				else if(Obj2[0] > Obj1[0])
				{
					gearDirection = 2;
					targeting_step = 3;
				}
				else
				{
					targeting_step = 9;
				}
				break;
			case 3:
				SmartDashboard::PutString("Feedback: ", "Move till strips are correct width according to height");
				if(!((Obj1[1] > (Obj1[0] * 2)) && (Obj2[1] > (Obj2[0] * 2))))
				{
					frc::SmartDashboard::PutNumber("GearDirection", gearDirection);
					switch(gearDirection)
					{
						case 1:
							gearPegAngleTarget(holding_angle, -.5, 0);
							break;
						case 2:
							gearPegAngleTarget(holding_angle, .5, 0);
							break;
					}
				}
				else
				{
					targeting_step = 4;
				}
				break;
			case 4:
				SmartDashboard::PutString("Feedback: ", "Align robot to peg");
				gearPegAngleTarget(holding_angle, (xLock - findTheLeft(Obj1[0], Obj2[0])) * kgearLining, 0);
				if((xLock - findTheLeft(Obj1[0], Obj2[0])) < 10)
				{
					targeting_step = 5;
				}
				break;
			case 5:
				SmartDashboard::PutString("Feedback: ", "Move forward towards target");
				gearPegAngleTarget(holding_angle, (xLock - findTheLeft(Obj1[0], Obj2[0])) * kgearLining, .25);
				break;
			case 6:
				gearPegAngleTarget(holding_angle, 0, 0);
				SmartDashboard::PutString("Feedback: ", "Done!!!!");
				break;
			case 7:
				SmartDashboard::PutString("Feedback: ", "Check if can see object");
				if((Obj1[0] == 0) && (Obj2[0] == 0))
				{
					SmartDashboard::PutString("Feedback: ", "No Target Visible");
				}
				else if(checkStep == 0)
				{
					targeting_step = 8;
				}
				else if(checkStep == 1)
				{
					targeting_step = 1;
				}
				else
				{
					targeting_step = 9;
				}
				break;
			case 8:
				SmartDashboard::PutString("Feedback: ", "Check if Too Close to Target");
				if(Obj1[2] > targetWidth - 5)
				{
					SmartDashboard::PutString("Feedback: ", "Too Close to Target");
				}
				else
				{
					targeting_step = 0;
				}
				break;
			case 9:
				SmartDashboard::PutString("Feedback: ", "UNEXPECTED ERROR!!!");
				break;
		}
	}
	;

	void TeleopInit() override
	{

	}
	;

	void TeleopPeriodic() override
	{
		float NegGyroAng = ahrs->GetFusedHeading();
		if(NegGyroAng > 180)
		{
			NegGyroAng = NegGyroAng - 360;
		}

		if(stick.GetRawButton(4))
		{
			m_robotDrive.MecanumDrive_Cartesian(0, 0, targetAdjustment(ahrs->GetFusedHeading(), 270), 0);
		}

		//Pixy Code
		PixyFunct();

		Obj1[0] = Pixyx1;
		Obj1[1] = Pixyy1;
		Obj1[2] = Pixyw1;
		Obj1[3] = Pixyh1;
		Obj2[0] = Pixyx2;
		Obj2[1] = Pixyy2;
		Obj2[2] = Pixyw2;
		Obj2[3] = Pixyh2;
		//Input Pixy Values
		frc::SmartDashboard::PutNumber("Targeting Step", targeting_step);
		if(stick.GetRawButton(3))
		{
			frc::SmartDashboard::PutString("Feedback: ", "Starting...");
			//PutGearOnPeg();
			PixyDriveTakeover();
		}
		else
		{
			targetHoldingAngle = false;
			checkStep = 0;
			targeting_step = 7;
		}

		if(stick.GetRawButton(7))
		{
			ahrs->Reset();
		}

		//Latch For Gyro
		if(xbox.GetRawButton(5) && xbox.GetRawButton(6) && !driveLatch)
		{
			driveToggle = !driveToggle;
			driveLatch = true;
		}
		else if(!xbox.GetRawButton(5) && !xbox.GetRawButton(6) && driveLatch)
		{
			driveLatch = false;
		}
		//Robot Centric
		if(driveToggle)
		{
			driveGyro = 0;
		}
		//Field Centric
		else if(!driveToggle)
		{
			driveGyro = ahrs->GetAngle();
		}

		//Drives the robot using the joystick, the gyro, and Mecanum
		//if the value of the stick is less than 10%, set to 0
		float deadZoneThreshold = 0.3;

		if((fabs(stick.GetRawAxis(0)) < deadZoneThreshold) || stick.GetRawButton(12))
		{
			joystickDeadBandX = 0;
		}
		//Otherwise set to joystick value
		else
		{
			joystickDeadBandX = stick.GetRawAxis(0);
		}

		//Repeat of above for Y
		if((fabs(stick.GetRawAxis(1)) < deadZoneThreshold) || stick.GetRawButton(11))
		{
			joystickDeadBandY = 0;
		}
		else
		{
			joystickDeadBandY = stick.GetRawAxis(1);
		}
		//Repeat of above for Z
		if(!stick.GetRawButton(12) && !stick.GetRawButton(11))//only turns when button 12 is pressed
		{
			gyroLatch = true;
			if(fabs(stick.GetRawAxis(2)) < .25)
			{
				joystickDeadBandZ = 0;
			}
			else
			{
				joystickDeadBandZ = -stick.GetRawAxis(2);
			}
		}
		else
		{

			if(gyroLatch)
			{
				gyroHeading = ahrs->GetAngle();
				gyroLatch = false;
			}

			joystickDeadBandZ = ((ahrs->GetAngle() - gyroHeading) / 180) * kgyroManip;

		}

		if(stick.GetRawButton(1))
		{
			joystickDeadBandX = stick.GetRawAxis(0) / 2;
			joystickDeadBandY = stick.GetRawAxis(1) / 2;
			if(!stick.GetRawButton(12))
			{
				joystickDeadBandZ = -stick.GetRawAxis(2) / 2;
			}
		}

		bool reset_yaw_button_pressed = stick.GetRawButton(6);
		if(reset_yaw_button_pressed)
		{
			ahrs->ZeroYaw();
		}
		try
		{
			// Use the joystick X axis for lateral movement,
			//Y axis for forward movement, and Z axis for rotation.
			//Use navX MXP yaw angle to define Field-centric transform
			if(!stick.GetRawButton(3) && !stick.GetRawButton(4))
			{
				m_robotDrive.MecanumDrive_Cartesian(joystickDeadBandX, joystickDeadBandY, joystickDeadBandZ, driveGyro + 180);
			}
		}
		catch(std::exception& ex)
		{
			std::string err_string = "Error communicating with Drive System:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		Wait(0.005); // wait 5ms to avoid hogging CPU cycles

		frc::SmartDashboard::PutNumber("Gyro", ahrs->GetAngle());
		frc::SmartDashboard::PutNumber("Fused Heading", ahrs->GetFusedHeading());
		frc::SmartDashboard::PutNumber("Virtual Fused Gyro", (ahrs->GetFusedHeading() - 360));

		//AHRS has a check PWM not push PWM so they can only be used for sensors

		//Latch for GO
		if(stick.GetRawButton(2) && !gearLatch)
		{
			gearToggle = !gearToggle;
			gearLatch = true;
		}
		else if(!stick.GetRawButton(2) && gearLatch)
		{
			gearLatch = false;
		}
		//Open Flaps
		if(gearToggle)
		{
			gearServoLeft.SetAngle(90);
			gearServoRight.SetAngle(0);
		}
		//Close Flaps
		else if(!gearToggle)
		{
			gearServoLeft.SetAngle(0);
			gearServoRight.SetAngle(90);
		}

		frc::SmartDashboard::PutBoolean("Gear Latch", gearLatch);

		//If button 3 is pressed, the ground intake motor will spin forwards at half power
		if(xbox.GetRawButton(1) == true)
		{
			groundIntakeMotor.SetSpeed(0.5);
		}
		//If button 4 is pressed, the ground intake motor will spin backwards at half power
		else if(xbox.GetRawButton(2) == true)
		{
			groundIntakeMotor.SetSpeed(-0.5);
		}
		//Otherwise, the ground intake motor will stop
		else
		{
			groundIntakeMotor.SetSpeed(0);
		}

		if(!gearTimeReset)
		{
			gearUSTimeStart = timer.Get();
			gearTimeReset = true;
			gearUSTrig.Set(false);
		}
		if((timer.Get() - gearUSTimeStart) == 0)
		{
			gearUSTrig.Set(true);

		}
		if(gearUSEcho.Get())
		{
			gearUSTrig.Set(false);
			gearTimeDelay = timer.Get() - gearUSTimeStart;
			gearTimeReset = false;
		}

		if(xbox.GetRawButton(3) == true)
		{
			//outakeEncoder.Set((outakeEncoder.GetSpeed()+.01)*2);
			outakeEncoder.Set(.25);
		}
		//If button 6 is pressed, the outake motor will spin backwards at half power until pot is less than 10
		else if(xbox.GetRawButton(4) == true)
		{
			outakeEncoder.Set(-.25);
		}
		//Otherwise, the outake motor will stop
		else
		{
			outakeEncoder.Set(0);
		}

		//if button 11 is pressed climber motor goes forward depending on how far the joystick is moved, only going forward proportionally to the absolute value of the joystick's y axis
		if(stick.GetRawButton(9))
		{
			//climberMotor.SetSpeed(-fabs(stick.GetRawAxis(3)));
			climberMotor.SetSpeed(-1);
		}
		else
		{
			if(stick.GetRawButton(10))
			{
				climberMotor.SetSpeed(-.5);
			}
			//if button is not pressed climber motor stops
			else
			{
				climberMotor.SetSpeed(0);
			}
		}

		frc::SmartDashboard::PutNumber("Left Servo Angle", gearServoLeft.GetAngle());
		frc::SmartDashboard::PutNumber("Right Servo Angle", gearServoRight.GetAngle());
		frc::SmartDashboard::PutNumber("rr encoder", rr.GetEncPosition());
		frc::SmartDashboard::PutNumber("rf pin a", rf.GetPinStateQuadA());
		frc::SmartDashboard::PutNumber("rf pin b", rf.GetPinStateQuadB());
		frc::SmartDashboard::PutNumber("lr encoder", lr.GetEncPosition());
		frc::SmartDashboard::PutNumber("lf encoder", lf.GetEncPosition());
		frc::SmartDashboard::PutNumber("rf encoder velocity", lf.GetEncVel());
		frc::SmartDashboard::PutNumber("rf encoder speed", lf.GetSpeed());
		frc::SmartDashboard::PutBoolean("Drive Toggle", driveToggle);
		frc::SmartDashboard::PutNumber("X Axis", stick.GetRawAxis(0));
		frc::SmartDashboard::PutNumber("Y Axis", stick.GetRawAxis(1));
		frc::SmartDashboard::PutNumber("Z Axis", stick.GetRawAxis(2));
		frc::SmartDashboard::PutNumber("Right Rear", rr.Get());
		frc::SmartDashboard::PutNumber("Right Front", rf.Get());
		frc::SmartDashboard::PutNumber("Left Rear", lr.Get());
		frc::SmartDashboard::PutNumber("Left Front", lf.Get());

	}

	void TestPeriodic() override
	{
		lw->Run();
	}
}
;

START_ROBOT_CLASS(Robot)
