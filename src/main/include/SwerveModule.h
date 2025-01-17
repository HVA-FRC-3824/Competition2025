#pragma once

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

struct WheelVector
{
    double Drive = 0.0;
    double Angle = 0.0;
};

class SwerveModule
{
    public:

        explicit     SwerveModule(int driveMotorCANid, int angleMotorCANid, int angleEncoderCANid);

        void         SetState(WheelVector wheelVector);

        WheelVector* GetWheelVector();

    private:

        // Private methods
        void   ConfigureDriveMotor(int driveMotorCANid);
        void   ConfigureAngleMotor(int angleMotorCANid, int angleEncoderCANid);

        void   OptimizeWheelAngle(WheelVector targetWheelVector, WheelVector *wheelVector);
        double ConvertAngleToTargetRange(WheelVector wheelVector);

        // Swerve vector struture (Drive and Angle)
        WheelVector                            m_wheelVector;

        // Swerve drive motor
        ctre::phoenix6::hardware::TalonFX     *m_driveMotor;
        
        // Swerve angle motor, encoder and PID controller
        rev::spark::SparkMax                  *m_angleMotor;
        rev::spark::SparkClosedLoopController *m_pidController;
        rev::spark::SparkRelativeEncoder      *m_angleEncoder;
        ctre::phoenix6::hardware::CANcoder    *m_angleAbsoluteEncoder;
};
