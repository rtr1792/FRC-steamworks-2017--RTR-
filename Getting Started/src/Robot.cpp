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


    static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
    static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
    static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
    static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
    static const int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
    static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;

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

	Robot() : lf(2), lr(1), rf(4), rr(5){
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

	AHRS *ahrs;
	// 0  2
	// 1  3

	//Ultrasonic Variables
	frc::DigitalOutput gearUSTrig { 0 };
	frc::DigitalInput gearUSEcho { 1 };
	float gearUSTimeStart = 0;
	bool gearTimeReset = 0;

	CANTalon lf; /*left front */
	CANTalon lr;/*left rear */
	CANTalon rf; /*right front */
	CANTalon rr; /*right rear */

	frc::RobotDrive m_robotDrive {lf, lr, rf, rr};
	frc::TalonSRX groundIntakeMotor { 1 }; //Motor for the ground intake
	frc::TalonSRX outakeMotor { 2 }; //Motor for the cloth lifting thing
	frc::TalonSRX climberMotor { 3 }; //Motor for climbing
	frc::Servo gearServoLeft { 4 };
	frc::Servo gearServoRight { 5 };
\


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

	//Temporary Value(s)
	float tempA = 0;



	frc::Timer timer;
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();


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
				//If the autocounter is less than 100, move forward and add 1 to the autocounter
				while(autocounter<AUTO_ONE)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, 1,0, 0);
					autocounter++;
					if(autocounter == AUTO_ONE)
					{
						m_robotDrive.MecanumDrive_Cartesian(0,0,0,0);
					}
				}
				//If the autocounter is equal to 100, stop the driving

				//turn a set degree
				while((ahrs->GetAngle()<ANGLE+WIGGLEROOM)||(ahrs->GetAngle()>ANGLE-WIGGLEROOM))
				{
					if(ahrs->GetAngle()<ANGLE+WIGGLEROOM)
					{
						m_robotDrive.MecanumDrive_Cartesian(0, 0,1, 0);
					}
					else
					{
						m_robotDrive.MecanumDrive_Cartesian(0, 0,-1, 0);
					}
				}
				//go field centric forward a set time (UNTIL_PEG_TIME)
				autocounter = 0;
				while(autocounter<AUTO_TWO)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, 1,0,ahrs->GetAngle());
			        autocounter++;
			        if(autocounter == AUTO_TWO)
			        {
			        	m_robotDrive.MecanumDrive_Cartesian(0,0,0,0);
			        }
				}

			//robot centric forward for te set time (UNTIL_GEAR_AT_PEG_TIME)
				while(autocounter<AUTO_THREE)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, 1,0,0);
					autocounter++;
					if(autocounter == AUTO_THREE)
					{
						m_robotDrive.MecanumDrive_Cartesian(0,0,0,0);
					}
				}
				//wait until us sensor says gear is out

				// go robot centric back a set time (UNTIL_OUT_OF_THE_WAY_TIME)
				while(autocounter<AUTO_FOUR)
				{
					m_robotDrive.MecanumDrive_Cartesian(0, -1,0,0);
					autocounter++;
					if(autocounter == AUTO_FOUR)
					{
						m_robotDrive.MecanumDrive_Cartesian(0,0,0,0);
					}
				}
			//go field centric forward a set time (BASE_LINE_TIME)
				 while(autocounter<AUTO_FIVE)
				 {
				 	m_robotDrive.MecanumDrive_Cartesian(0, -1,0,ahrs->GetAngle());
				 	autocounter++;
				 	if(autocounter == AUTO_FIVE)
				 	{
				 		m_robotDrive.MecanumDrive_Cartesian(0,0,0,0);
				 	}
				 }
				};

	void TeleopInit() override {

	};

	void TeleopPeriodic() override {



		//Drives the robot using the joystick, the gyro, and Mecanum
		//if the value of the stick is less than 10%, set to 0
		if (abs(stick.GetX()) < .1)
		{
			joystickDeadBandX = 0;
		}
		//Otherwise set to joystick value
		else
		{
			joystickDeadBandX = stick.GetX();
		}
		//Repeat of above for Y
		if (abs(stick.GetY()) < .1)
		{
			joystickDeadBandY = 0;
		}

		else
		{
			joystickDeadBandY = -stick.GetY();
		}
		//Repeat of above for Z
		if (abs(stick.GetZ()) < .1)
		{
			joystickDeadBandZ = 0;
		}

		else
		{
			joystickDeadBandZ = stick.GetZ();
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



		//Set values to each motor
		//frontLeftMotor.SetSpeed(frontLeft);
		//frontRightMotor.SetSpeed(-frontRight);
		//rearLeftMotor.SetSpeed(rearLeft);
		//rearRightMotor.SetSpeed(-rearRight);


		bool reset_yaw_button_pressed = stick.GetRawButton(1);
		if (reset_yaw_button_pressed) {
			ahrs->ZeroYaw();
		}
		try {
			/* Use the joystick X axis for lateral movement,
			 Y axis for forward movement, and Z axis for rotation.
			 Use navX MXP yaw angle to define Field-centric transform */
			m_robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(),
					stick.GetZ(),driveGyro);
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
			//open the flaps
		}
		//Close Flaps
		else if(!gearToggle)
		{
			//close the flaps
		}



		//If button 3 is pressed, the ground intake motor will spin forwards at half power
		if(stick.GetRawButton(3) == true)
		{
			groundIntakeMotor.SetSpeed(0.5);
		}
		//If button 4 is pressed, the ground intake motor will spin backwards at half power
		else if(stick.GetRawButton(4) == true)
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
			//timer.Get()=gearUSTimeStart;
			gearTimeReset = true;
		}
		if((timer.Get()-gearUSTimeStart)==0)
		{
			gearUSTrig.Set(false);
		}
        else if((timer.Get()-gearUSTimeStart)==2)



		//If button 5 is pressed, the outake motor will spin forwards at half power until pot is greater than 40
		if(stick.GetRawButton(5) == true)
		{
			outakeMotor.SetSpeed(0.5);
		}
		//If button 6 is pressed, the outake motor will spin backwards at half power until pot is less than 10
		else if(stick.GetRawButton(6) == true)
		{
			outakeMotor.SetSpeed(-0.5);
		}
		//Otherwise, the outake motor will stop
		else
		{
			outakeMotor.SetSpeed(0);
		}


		//if button 11 is pressed climber motor goes forward depending on how far the joystick is moved, only going forward proportionally to the absolute value of the joystick's y axis
		if(stick.GetRawButton(11))
		{
			climberMotor.SetSpeed(-fabs(stick.GetY()));
		}
		//if button is not pressed climber motor stops
		else
		{
			climberMotor.SetSpeed(0);
		}

		if(stick.GetRawButton(7))
		{
			gearServoLeft.SetAngle(20);
		}

		else if(stick.GetRawButton(8))
		{
			gearServoLeft.SetAngle(-20);
		}

		frc::SmartDashboard::PutNumber("Servo Angle",gearServoLeft.GetAngle());


		//frc::SmartDashboard::PutNumber("height of gear",gearUlt.GetRangeInches());

		//Getting Button 5 output
		//frc::SmartDashboard::PutBoolean("Button 5",stick.GetRawButton(5));

//		frc::SmartDashboard::PutNumber("Distance in millimeters", ultrasonicTest.GetRangeMM());


	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
