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

#define POT_MAX_VALUE 40
#define POT_MIN_VALUE 10
#define ANGLE 45
#define WIGGLEROOM .5
#define MXP_IO_VOLTAGE (double)3.3f /* Alternately, 5.0f   */
#define MIN_AN_TRIGGER_VOLTAGE (double)0.76f
#define MAX_AN_TRIGGER_VOLTAGE MXP_IO_VOLTAGE - (double)2.0f

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

	Robot() {
		//Generic initialization code
		myRobot.SetExpiration(0.1);
		//Inverts the left side
		myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		timer.Start();  //Initializes the timer
		autocounter = 0;

		pwm_out_0 = new Victor(        GetChannelFromPin( PinType::PWMs,       0 ));


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
	//

	/*
	//Talons for testing the PWM output
	frc::TalonSRX testTalon0 { 0 };
	frc::TalonSRX testTalon1 { 1 };
	frc::TalonSRX testTalon2 { 2 };
	frc::TalonSRX testTalon3 { 3 };
	frc::TalonSRX testTalon4 { 4 };
	frc::TalonSRX testTalon5 { 5 };
	frc::TalonSRX testTalon6 { 6 };
	frc::TalonSRX testTalon7 { 7 };
	frc::TalonSRX testTalon8 { 8 };
	frc::TalonSRX testTalon9 { 9 };
	 */

	/*
	//Buttons for testing DIO input
	frc::DigitalInput testButton0 { 0 };
	frc::DigitalInput testButton1 { 1 };
	frc::DigitalInput testButton2 { 2 };
	frc::DigitalInput testButton3 { 3 };
	frc::DigitalInput testButton4 { 4 };
	frc::DigitalInput testButton5 { 5 };
	frc::DigitalInput testButton6 { 6 };
	frc::DigitalInput testButton7 { 7 };
	frc::DigitalInput testButton8 { 8 };
	frc::DigitalInput testButton9 { 9 };
	*/

	/*
	//LEDs for testing DIO Output
	frc::DigitalOutput testLED0 { 0 };
	frc::DigitalOutput testLED1 { 1 };
	frc::DigitalOutput testLED2 { 2 };
	frc::DigitalOutput testLED3 { 3 };
	frc::DigitalOutput testLED4 { 4 };
	frc::DigitalOutput testLED5 { 5 };
	frc::DigitalOutput testLED6 { 6 };
	frc::DigitalOutput testLED7 { 7 };
	frc::DigitalOutput testLED8 { 8 };
	frc::DigitalOutput testLED9 { 9 };
	*/

	//Potentiometers for testing PWM input
//	frc::PWM testPot0 { 0 };
//	frc::PWM testPot1 { 1 };
//	frc::PWM testPot2 { 2 };
//	frc::PWM testPot3 { 3 };
//	frc::PWM testPot4 { 4 };
//	frc::PWM testPot5 { 5 };
//	frc::PWM testPot6 { 6 };
//	frc::Potentiometer testPot7 { 7 };
//	frc::Potentiometer testPot8 { 8 };
//	frc::Potentiometer testPot9 { 9 };





	frc::Joystick stick { 0 };         // Only joystick


	AHRS *ahrs;
	// 0  2
	// 1  3

	/*
	//Motors for Driving
	TalonSRX frontLeftMotor { 0 };
	TalonSRX frontRightMotor { 1 };
	TalonSRX rearLeftMotor { 2 };
	TalonSRX rearRightMotor { 3 };
	*/

	frc::RobotDrive myRobot {1,2,3,4};
	frc::TalonSRX groundIntakeMotor { 5}; //Motor for the ground intake
	frc::TalonSRX outakeMotor { 6 }; //Motor for the cloth lifting thing
	frc::TalonSRX climberMotor { 7 }; //Motor for climbing

	Victor *pwm_out_0 ;



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
			while(autocounter<100)
			{
				myRobot.MecanumDrive_Cartesian(0, 1,0, 0);
				autocounter++;
				if(autocounter == 100)
				{
					myRobot.MecanumDrive_Cartesian(0,0,0,0);
				}
			}
			//If the autocounter is equal to 100, stop the driving

			//turn a set degree
			while((ahrs->GetAngle()<ANGLE+WIGGLEROOM)||(ahrs->GetAngle()>ANGLE-WIGGLEROOM))
			{
				if(ahrs->GetAngle()<ANGLE+WIGGLEROOM)
				{
					myRobot.MecanumDrive_Cartesian(0, 0,1, 0);
				}
				else
				{
					myRobot.MecanumDrive_Cartesian(0, 0,-1, 0);
				}
			}
			//go field centric forward a set time (UNTIL_PEG_TIME)
			autocounter = 0;
			while(autocounter<100)
			{
				myRobot.MecanumDrive_Cartesian(0, 1,0,ahrs->GetAngle());
		        autocounter++;
		        if(autocounter == 100)
		        {
		        	myRobot.MecanumDrive_Cartesian(0,0,0,0);
		        }
			}

		//robot centric forward for te set time (UNTIL_GEAR_AT_PEG_TIME)
			while(autocounter<100)
			{
				myRobot.MecanumDrive_Cartesian(0, 1,0,0);
				autocounter++;
				if(autocounter == 100)
				{
					myRobot.MecanumDrive_Cartesian(0,0,0,0);
				}
			}
			//wait until us sensor says gear is out

			// go robot centric back a set time (UNTIL_OUT_OF_THE_WAY_TIME)
			while(autocounter<100)
			{
				myRobot.MecanumDrive_Cartesian(0, -1,0,0);
				autocounter++;
				if(autocounter == 100)
				{
					myRobot.MecanumDrive_Cartesian(0,0,0,0);
				}
			}
		//go field centric forward a set time (BASE_LINE_TIME)
			 while(autocounter<100)
			 {
			 	myRobot.MecanumDrive_Cartesian(0, -1,0,ahrs->GetAngle());
			 	autocounter++;
			 	if(autocounter == 100)
			 	{
			 		myRobot.MecanumDrive_Cartesian(0,0,0,0);
			 	}
			 }
			};

	void TeleopInit() override {

	};

	void TeleopPeriodic() override {


		//These functions are just for testing purposes

		/*
		//Testing PWM Output
		if(stick.GetRawButton(5))//If button 5 is pushed, motors runs forward
		{
			testTalon0.SetSpeed(0.25); //Motors go forward at 25% speed
			testTalon1.SetSpeed(0.25);
			testTalon2.SetSpeed(0.25);
			testTalon3.SetSpeed(0.25);
			testTalon4.SetSpeed(0.25);
			testTalon5.SetSpeed(0.25);
			testTalon6.SetSpeed(0.25);
			testTalon7.SetSpeed(0.25);
			testTalon8.SetSpeed(0.25);
			testTalon9.SetSpeed(0.25);
		}
		else //If button is not pressed
		{
			testTalon0.SetSpeed(0); //Stop motors
			testTalon1.SetSpeed(0);
			testTalon2.SetSpeed(0);
			testTalon3.SetSpeed(0);
			testTalon4.SetSpeed(0);
			testTalon5.SetSpeed(0);
			testTalon6.SetSpeed(0);
			testTalon7.SetSpeed(0);
			testTalon8.SetSpeed(0);
			testTalon9.SetSpeed(0);
		}
		*/

		/*
		//Testing the DIOs input by presenting to the SmartDashboard
		frc::SmartDashboard::PutBoolean("Button 0",testButton0.Get());
		frc::SmartDashboard::PutBoolean("Button 1",testButton1.Get());
		frc::SmartDashboard::PutBoolean("Button 2",testButton2.Get());
		frc::SmartDashboard::PutBoolean("Button 3",testButton3.Get());
		frc::SmartDashboard::PutBoolean("Button 4",testButton4.Get());
		frc::SmartDashboard::PutBoolean("Button 5",testButton5.Get());
		frc::SmartDashboard::PutBoolean("Button 6",testButton6.Get());
		frc::SmartDashboard::PutBoolean("Button 7",testButton7.Get());
		frc::SmartDashboard::PutBoolean("Button 8",testButton8.Get());
		frc::SmartDashboard::PutBoolean("Button 9",testButton9.Get());
		*/

		/*
		//Testing DIOs ouput by turning on LEDs
		//If button is pushed, do below
		if(stick.GetRawButton(5))
		{
			//Turn on LEDs
			testLED0.Set(1);
			testLED1.Set(1);
			testLED2.Set(1);
			testLED3.Set(1);
			testLED4.Set(1);
			testLED5.Set(1);
			testLED6.Set(1);
			testLED7.Set(1);
			testLED8.Set(1);
			testLED9.Set(1);
		}
		else//If button is not pushed, do below
		{
			//Turn off LEDs
			testLED0.Set(0);
			testLED1.Set(0);
			testLED2.Set(0);
			testLED3.Set(0);
			testLED4.Set(0);
			testLED5.Set(0);
			testLED6.Set(0);
			testLED7.Set(0);
			testLED8.Set(0);
			testLED9.Set(0);
		}
		*/

		//Sets the variable gyroAngle to the value that the Gyro has
		//gyroAngle = gyro.GetRaw();
		//gyroAngle = 5;
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

		//MATH!!! Copied code from Chief Delphi. Member Ether
		frontLeft = joystickDeadBandY + joystickDeadBandZ + joystickDeadBandX;
		frontRight = joystickDeadBandY - joystickDeadBandZ - joystickDeadBandX;
		rearLeft = joystickDeadBandY + joystickDeadBandZ - joystickDeadBandX;
		rearRight = joystickDeadBandY - joystickDeadBandZ + joystickDeadBandX;


		//Somehow determines a max
		driveMax = abs(frontLeft);
		tempA = abs(frontRight);
		if(tempA > driveMax)
		{
			driveMax = tempA;
		}
		tempA = abs(rearLeft);
		if(tempA > driveMax)
		{
			driveMax = tempA;
		}
		tempA = abs(rearRight);
		if(tempA > driveMax)
		{
			driveMax = tempA;
		}

		if(driveMax >1)
		{
			frontLeft /= driveMax;
			frontRight /= driveMax;
			rearLeft /= driveMax;
			rearRight /= driveMax;
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
			/* Use the joystick X axis for lateral movement,            */
			/* Y axis for forward movement, and Z axis for rotation.    */
			/* Use navX MXP yaw angle to define Field-centric transform */
			myRobot.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(),
					stick.GetZ(), ahrs->GetAngle());
		} catch (std::exception& ex) {
			std::string err_string = "Error communicating with Drive System:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		Wait(0.005); // wait 5ms to avoid hogging CPU cycles

		frc::SmartDashboard::PutNumber("Gyro",ahrs->GetAngle());

		//AHRS has a check PWM not push PWM so they can only be used for sensors




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
			climberMotor.SetSpeed(fabs(stick.GetY()));
		}
		//if button is not pressed climber motor stops
		else
		{
			climberMotor.SetSpeed(0);
		}

		//frc::SmartDashboard::PutNumber("height of gear",gearUlt.GetRangeInches());

		//Getting Button 5 output
		//frc::SmartDashboard::PutBoolean("Button 5",stick.GetRawButton(5));


	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
