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
#include "TPixy.h"
#include "PixyI2C.h"

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


    static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
    static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
    static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
    static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
    static const int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
    static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;

    typedef unsigned char byte;

enum PinType { DigitalIO, PWMs, AnalogIn, AnalogOut };
int GetChannelFromPin( PinType type, int io_pin_number ) {
    int roborio_channel = 0;
    if ( io_pin_number < 0 ) {
        throw std::runtime_error("Error:  navX MXP I/O Pin #");
    }
    switch ( type ) {
    case DigitalIO:
        if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
            throw std::runtime_error("Error:  Invalid navX MXP Digital I/O Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS +
                          (io_pin_number > 3 ? 4 : 0);
        break;
    case PWMs:
        if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
            throw std::runtime_error("Error:  Invalid navX MXP Digital I/O Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
        break;
    case AnalogIn:
        if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
            throw new std::runtime_error("Error:  Invalid navX MXP Analog Input Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
        break;
    case AnalogOut:
        if ( io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER ) {
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
class Robot: public frc::IterativeRobot {

public:

	Robot() : rr(1), rf(2), lr(4), lf(5), outakeEncoder(3){


		i2cthing = new I2C(I2C::kOnboard, I2C_SLAVE_ADR);
		//Generic initialization code
		m_robotDrive.SetExpiration(0.1);
		//Inverts the left side
		m_robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		m_robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		timer.Start();  //Initializes the timer
		autocounter = 0;


		//This try catch block checks to see if an error instantiating the navx is thrown
		try {
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
		        } catch (std::exception& ex ) {
		            std::string err_string = "Error instantiating navX MXP:  ";
		            err_string += ex.what();
		            DriverStation::ReportError(err_string.c_str());
		        }
		        if ( ahrs ) {
		            LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		        }


	}

private:





	frc::Joystick stick { 0 };         // Only joystick
	frc::XboxController xbox { 1 };

	double gyroHeading = 0;
	bool gyroLatch= false;
	float kgyroManip = 1;

	AHRS *ahrs;
	// 0  2
	// 1  3

	//Ultrasonic Variables
	frc::Ultrasonic US { 0 , 1 };
	frc::DigitalOutput gearUSTrig { 0 };
	frc::DigitalInput gearUSEcho { 1 };
	float gearUSTimeStart = 0;
	bool gearTimeReset = 0;
	float gearTimeDelay = 0;

	/*frc::AnalogInput ultrasonicTest { 0 };
	const int Vcc = 5;
	const double Vm = null; */

	//double Vi = Vcc/512;

	TPixy<LinkI2C> Pixy;
	float Pixyx1 = 0;
	float Pixyy1 = 0;
	float Pixyh1 = 0;
	float Pixyw1 = 0;
	float PixyBlockNumb1 = 0;
	float PixySig1 = 0;
	float Pixyx2 = 0;
	float Pixyy2 = 0;
	float Pixyh2 = 0;
	float Pixyw2 = 0;
	float PixyBlockNumb2 = 0;
	float PixySig2 = 0;

	I2C *i2cthing;

	CANTalon rr; /*left front */
	CANTalon rf;/*left rear */
	CANTalon lr; /*right front */
	CANTalon lf; /*right rear */
	CANTalon outakeEncoder; /*outake Encoder */

	frc::RobotDrive m_robotDrive {rr, rf, lr, lf};
	frc::TalonSRX groundIntakeMotor { 2 }; //Motor for the ground intake
	frc::TalonSRX outakeMotor { 3 }; //Motor for the cloth lifting thing
	frc::TalonSRX climberMotor { 4 }; //Motor for climbing
	frc::Servo gearServoLeft { 0 };
	frc::Servo gearServoRight { 1 };



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
	float driveGyro = 0;
	bool driveLatch = false;

	//GO Latch Variables
	bool gearToggle = false;
    bool gearLatch = false;
    //bool gearServoLatch = false;

	//Temporary Value(s)
	float tempA = 0;



	frc::Timer timer;
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();

	std::unique_ptr<frc::Command> autocommand;
	frc::SendableChooser<frc::Command*> chooser;

	void RobotInit() override {

		//chooser.AddDefault("Do Nothing", new Pickup());
	}

	void AutonomousInit() override {
		timer.Reset();
		timer.Start();

//		std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("datatable");
//		lw = LiveWindow::GetInstance();
//		try {
//			/* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
//			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
//			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
//			ahrs = new AHRS(SerialPort::Port::kMXP);
//		} catch (std::exception ex) {
//			std::string err_string = "Error instantiating navX-MXP:  ";
//			err_string += ex.what();
//			DriverStation::ReportError(err_string.c_str());
//		}
//		if (ahrs) {
//			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
//		}
	}
	int autocounter;
	void AutonomousPeriodic() override
		{
			if(timer.Get()<3)
			{
				m_robotDrive.MecanumDrive_Cartesian(0,.25,0,0);
			}
		};

	void PixyFunct()
	{
		byte buff[31];
		int translate[15];
		i2cthing->Read(0x64, 31, buff);


		if(!(buff[1]==buff[2]))
		{
			if(buff[0]==0)
			{
				for(int i = 0;i<30;i++)
				{
					buff[i]=buff[i+1];
				}
			}
		}
		for(int i = 0;i<15;i++)
		{
			translate[i]=buff[(2*i)+1]*256 + buff[2*i];
		}

		SmartDashboard::PutNumber("Buff0",buff[0]);
		SmartDashboard::PutNumber("Buff1",buff[1]);
		SmartDashboard::PutNumber("Buff2",buff[2]);
		SmartDashboard::PutNumber("Buff3",buff[3]);
		SmartDashboard::PutNumber("Buff4",buff[4]);
		SmartDashboard::PutNumber("Buff5",buff[5]);
		SmartDashboard::PutNumber("Buff6",buff[6]);
		SmartDashboard::PutNumber("Buff7",buff[7]);
		SmartDashboard::PutNumber("Buff8",buff[8]);
		SmartDashboard::PutNumber("Buff9",buff[9]);
		SmartDashboard::PutNumber("Buff10",buff[10]);
		SmartDashboard::PutNumber("Buff11",buff[11]);
		SmartDashboard::PutNumber("Buff12",buff[12]);
		SmartDashboard::PutNumber("Buff13",buff[13]);


		SmartDashboard::PutNumber("Translate0",translate[0]);
		SmartDashboard::PutNumber("Translate1",translate[1]);
		SmartDashboard::PutNumber("Translate2",translate[2]);
		SmartDashboard::PutNumber("Translate3",translate[3]);
		SmartDashboard::PutNumber("Translate4",translate[4]);
		SmartDashboard::PutNumber("Translate5",translate[5]);
		SmartDashboard::PutNumber("Translate6",translate[6]);
		SmartDashboard::PutNumber("Translate7",translate[7]);
		SmartDashboard::PutNumber("Translate8",translate[8]);
		SmartDashboard::PutNumber("Translate9",translate[9]);
		SmartDashboard::PutNumber("Translate10",translate[10]);
		SmartDashboard::PutNumber("Translate11",translate[11]);
		SmartDashboard::PutNumber("Translate12",translate[12]);
		SmartDashboard::PutNumber("Translate13",translate[13]);
		SmartDashboard::PutNumber("Translate14",translate[14]);


	};

	void TeleopInit() override {

	};

	void TeleopPeriodic() override {

		//Pixy Code
		PixyFunct();

		//Pixy.GetStart();

		Pixy.GetStart();
		PixyBlockNumb1 = Pixy.GetBlocks(2);
		Pixyx1 = Pixy.blocks->x;
		Pixyy1 = Pixy.blocks->y;
		Pixyh1 = Pixy.blocks->height;
		Pixyw1 = Pixy.blocks->width;
		//PixyBlockNumb2 = Pixy.link.getByte();
		PixySig1 = Pixy.blocks->signature;
		Pixyx2 = Pixy.blocks->x;
		Pixyy2 = Pixy.blocks->y;
		Pixyh2 = Pixy.blocks->height;
		Pixyw2 = Pixy.blocks->width;
		PixySig2 = Pixy.blocks->signature;

		frc::SmartDashboard::PutNumber("X 1", Pixyx1);
		frc::SmartDashboard::PutNumber("Y 1", Pixyy1);
		frc::SmartDashboard::PutNumber("H 1", Pixyh1);
		frc::SmartDashboard::PutNumber("W 1", Pixyw1);
		frc::SmartDashboard::PutNumber("Sig 1", PixySig1);
		frc::SmartDashboard::PutNumber("Block Numb 1", PixyBlockNumb1);
		frc::SmartDashboard::PutNumber("X 2", Pixyx2);
		frc::SmartDashboard::PutNumber("Y 2", Pixyy2);
		frc::SmartDashboard::PutNumber("H 2", Pixyh2);
		frc::SmartDashboard::PutNumber("W 2", Pixyw2);
		frc::SmartDashboard::PutNumber("Sig 2", PixySig2);
		frc::SmartDashboard::PutNumber("Block Numb 2", PixyBlockNumb2);
		frc::SmartDashboard::PutNumber("Make Sure Updating", 33);



				if(stick.GetRawButton(7))
				{
					ahrs->Reset();
				}

		//Latch For Gyro
				if(xbox.GetRawButton(5)&&xbox.GetRawButton(6)&&!driveLatch)
				{
						driveToggle=!driveToggle;
						driveLatch = true;
				}
				else if(!xbox.GetRawButton(5)&&!xbox.GetRawButton(6)&&driveLatch)
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

		if ((fabs(stick.GetRawAxis(0)) < deadZoneThreshold)||stick.GetRawButton(12))
		{
			joystickDeadBandX = 0;
		}
		//Otherwise set to joystick value
		else
		{
			joystickDeadBandX = stick.GetRawAxis(0);
		}

		//Repeat of above for Y
		if ((fabs(stick.GetRawAxis(1)) < deadZoneThreshold)||stick.GetRawButton(11))
		{
			joystickDeadBandY = 0;
		}
		else
		{
			joystickDeadBandY = stick.GetRawAxis(1);
		}
		//Repeat of above for Z
		if(!stick.GetRawButton(12)&&!stick.GetRawButton(11))   //only turns when button 12 is pressed
		{
			gyroLatch = true;
			if (fabs(stick.GetRawAxis(2)) < .25)
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
				gyroHeading=ahrs->GetAngle();
				gyroLatch = false;
			}



			joystickDeadBandZ = ((ahrs->GetAngle()-gyroHeading)/180)*kgyroManip;

		}


		if(stick.GetRawButton(1))
		{
			joystickDeadBandX=joystickDeadBandX/2;
			joystickDeadBandY=joystickDeadBandY/2;
			if(!stick.GetRawButton(12))
			{
				joystickDeadBandZ=joystickDeadBandZ/2;
			}
		}



		//Set values to each motor
		//frontLeftMotor.SetSpeed(frontLeft);
		//frontRightMotor.SetSpeed(-frontRight);
		//rearLeftMotor.SetSpeed(rearLeft);
		//rearRightMotor.SetSpeed(-rearRight);



		bool reset_yaw_button_pressed = stick.GetRawButton(6);
		if (reset_yaw_button_pressed) {
			ahrs->ZeroYaw();
		}
		try {
			// Use the joystick X axis for lateral movement,
			 //Y axis for forward movement, and Z axis for rotation.
			 //Use navX MXP yaw angle to define Field-centric transform
			m_robotDrive.MecanumDrive_Cartesian(joystickDeadBandX,joystickDeadBandY,
					joystickDeadBandZ,driveGyro+180);
		} catch (std::exception& ex) {
			std::string err_string = "Error communicating with Drive System:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		Wait(0.005); // wait 5ms to avoid hogging CPU cycles

		frc::SmartDashboard::PutNumber("Gyro",ahrs->GetAngle());

		//AHRS has a check PWM not push PWM so they can only be used for sensors


		//Latch for GO
		if(stick.GetRawButton(2)&&!gearLatch)
		{
			gearToggle=!gearToggle;
			gearLatch=true;
		}
		else if(!stick.GetRawButton(2)&&gearLatch)
		{
			gearLatch = false;
		}
		//Open Flaps
		if(gearToggle)
		{
			gearServoLeft.SetAngle(90);
			gearServoRight.SetAngle(90);
		}
		//Close Flaps
		else if(!gearToggle)
		{
			gearServoLeft.SetAngle(0);
			gearServoRight.SetAngle(180);
		}

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
		if((timer.Get()-gearUSTimeStart)==0)
		{
			gearUSTrig.Set(true);

		}
        if(gearUSEcho.Get())
        {
        	gearUSTrig.Set(false);
        	gearTimeDelay = timer.Get()-gearUSTimeStart;
        	gearTimeReset = false;
        }





		//If button 5 is pressed, the outake motor will spin forwards at half power until pot is greater than 40
       /*int OutULimit = 4;
		int OutLLimit = 0;
        if(((xbox.GetRawButton(3) == false)&&(xbox.GetRawButton(4) == false))||outakeEncoder.GetEncPosition>OutULimit||outakeEncoder.GetEncPosition<OutLLimit)
        {
        	outakeEncoder.SetSpeed(0);
        }
        */
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
		else if(stick.GetRawButton(10))
		{
			climberMotor.SetSpeed(-.5);
		}
		//if button is not pressed climber motor stops
		else
		{
			climberMotor.SetSpeed(0);
		}


		/*
		if(stick.GetRawButton(4)&&!gearServoLatch)
		{
			gearServoLeft.SetAngle(gearServoLeft.GetAngle()-1);
			gearServoLatch = true;
		}
		else if(stick.GetRawButton(6)&&!gearServoLatch)
		{
			gearServoLeft.SetAngle(gearServoLeft.GetAngle()+1);
			gearServoLatch = true;
		}
		else if(stick.GetRawButton(3)&&!gearServoLatch)
		{
			gearServoRight.SetAngle(gearServoRight.GetAngle()-1);
			gearServoLatch = true;
		}
		else if(stick.GetRawButton(5)&&!gearServoLatch)
		{
			gearServoRight.SetAngle(gearServoRight.GetAngle()+1);
			gearServoLatch = true;
		}
		else if(!stick.GetRawButton(4)&&!stick.GetRawButton(3)&&!stick.GetRawButton(5)&&!stick.GetRawButton(6)&&gearServoLatch)
		{
			gearServoLatch = false;
		}
		*/



		frc::SmartDashboard::PutNumber("Left Servo Angle",gearServoLeft.GetAngle());
		frc::SmartDashboard::PutNumber("Right Servo Angle",gearServoRight.GetAngle());
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
		frc::SmartDashboard::PutNumber("Z Axis",stick.GetRawAxis(2));
		frc::SmartDashboard::PutNumber("Right Rear",rr.Get());
		frc::SmartDashboard::PutNumber("Right Front",rf.Get());
		frc::SmartDashboard::PutNumber("Left Rear",lr.Get());
		frc::SmartDashboard::PutNumber("Left Front",lf.Get());
		//frc::SmartDashboard::PutNumber("height of gear",gearUlt.GetRangeInches());

		//Getting Button 5 output
		//frc::SmartDashboard::PutBoolean("Button 5",stick.GetRawButton(5));


		//frc::SmartDashboard::PutNumber("Distance in millimeters", ultrasonicTest.GetValue());


	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
