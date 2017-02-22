#include "driveTrain.h"

driveTrain::driveTrain() :
frc::Subsystem("driveTrain"){

}
void driveTrain::Drive(float x, float y, float z, float gyro){
	robotDrive.MecanumDrive_Cartesian(x,y,z,gyro);
}
