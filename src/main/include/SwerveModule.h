#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <rev/SparkMax.h>

#include "Constants.h"

struct WheelVector
{
    double Drive = 0.0;
    double Angle = 0.0;
};

class SwerveModule
{
    public:

        explicit               SwerveModule(int driveMotorCANid, int angleMotorCANid, int angleEncoderCanId);

        void                   SetState(WheelVector wheelVector);

        void                   SetWheelAngleToForward(units::angle::degree_t desiredAngle);

        units::angle::degree_t GetAbsoluteAngle();

        WheelVector*           GetWheelVector();

        double                 GetSwerveAngle();

    private:

        // Private methods
        void   ConfigureDriveMotor();
        void   ConfigureAngleMotor();

        void   OptimizeWheelAngle(WheelVector targetWheelVector, WheelVector *wheelVector);
        double ConvertAngleToTargetRange(WheelVector wheelVector);

        // Swerve vector struture (Drive and Angle)
        WheelVector                           m_wheelVector{0.0, 0.0};

        // Swerve drive motor
        ctre::phoenix6::hardware::TalonFX     m_driveMotor;
        ctre::phoenix6::controls::VoltageOut  m_voltageOut{0_V};  // Controller mode is VoltageOut

        // Swerve angle motor, encoder and PID controller
        rev::spark::SparkMax                  m_angleMotor;
        rev::spark::SparkClosedLoopController m_pidController;
        rev::spark::SparkRelativeEncoder      m_angleEncoder;
        ctre::phoenix6::hardware::CANcoder    m_angleAbsoluteEncoder;
};
