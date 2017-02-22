#ifndef driveTrain_H
#define driveTrain_H

#include <RobotDrive.h>
#include "CANTalon.h"
#include "AHRS.h"

namespace frc {
class Joystick;
}

class driveTrain: public frc::Subsystem{
public:
	driveTrain();

	void Drive(float xaxis, float yaxis, float zaxis, float gyro);

private:
	CANTalon rr { 1 };
	CANTalon rf { 2 };
	CANTalon lr { 4 };
	CANTalon lf { 5 };
	RobotDrive robotDrive {rr, rf, lr, lf};
	AHRS *ahrs = new AHRS(SPI::Port::kMXP);
};

#endif
