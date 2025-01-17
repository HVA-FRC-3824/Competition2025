#pragma once

#include <cmath>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "SwerveModule.h"

class Drivetrain : public frc2::SubsystemBase
{
    public:

        explicit     Drivetrain();

        void         Drive(double forward, double strafe, double angle, double gyro);

        void         SetFieldCentricity(bool fieldCentric);
        bool         GetFieldCentricity();
        
        WheelVector* GetSwerveModuleWheelVector(int swerveModuleIndex);

    private:

        // Private methods
        void FieldCentricAngleConversion(double *forward, double *strafe, double angle);
        void CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, WheelVector wheelVector[]);
        void NormalizeSpeed(WheelVector wheelVector[]);

        bool   m_fieldCentricity = false;

        double R = sqrt((ChassisConstants::kChassisLength * ChassisConstants::kChassisLength) + 
                        (ChassisConstants::kChassisWidth  * ChassisConstants::kChassisWidth));

        SwerveModule *m_swerveModule[ChassisConstants::kNumberOfSwerveModules];  // Pointers to the four swerve modules
};
