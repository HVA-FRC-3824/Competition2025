#include "SwerveModule.h"

#pragma region SwerveModule Contructor
/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) :
                           m_driveMotor(driveMotorCanId, CanConstants::CanBus),
                           m_angleMotor(angleMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
                           m_angleAbsoluteEncoder(angleEncoderCanId, CanConstants::CanBus),
                           m_angleEncoder(m_angleMotor.GetEncoder()),
                           m_turnClosedLoopController(m_angleMotor.GetClosedLoopController())

{
    // Configure the drive and angle motors
    ConfigureDriveMotor();
    ConfigureAngleMotor();

    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetPosition(0_tr);
}
#pragma endregion

#pragma region ConfigureDriveMotor
/// @brief Method to configure the drive motor.
void SwerveModule::ConfigureDriveMotor()
{
    // Create the drive motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};

    // Add the "Motor Output" section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Add the "Current Limits" section settings
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit       = SwerveConstants::DriveMaxAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Add the "Slot0" section settings
    ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kP = SwerveConstants::DriveP;
    slot0Configs.kI = SwerveConstants::DriveI;
    slot0Configs.kD = SwerveConstants::DriveD;
    slot0Configs.kV = SwerveConstants::DriveV;
    slot0Configs.kA = SwerveConstants::DriveA;

    // Add the "Feedback" section settings
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = talonFXConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = SwerveConstants::WheelCircumference.value() / SwerveConstants::DriveMotorReduction;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CanConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_driveMotor.GetConfigurator().Apply(talonFXConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl;
}
#pragma endregion

#pragma region ConfigureAngleMotor
/// @brief Method to configure the angle motor and encoder.
void SwerveModule::ConfigureAngleMotor()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(SwerveConstants::AngleMaxAmperage);
    sparkMaxConfig.encoder
        //.Inverted(true)
        .PositionConversionFactor(SwerveConstants::AngleRadiansToMotorRevolutions)
        .VelocityConversionFactor(SwerveConstants::AngleRadiansToMotorRevolutions / 60.0);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(SwerveConstants::AngleP, SwerveConstants::AngleI, SwerveConstants::AngleD)
        //.OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true);
        //.PositionWrappingInputRange(0, 2 * std::numbers::pi);  TODO: Try these settings

    // Write the configuration to the motor controller
    m_angleMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region SetDesiredState
/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(frc::SwerveModuleState& desiredState, std::string description)
{
    frc::SmartDashboard::PutNumber(description + "Drive", (double) desiredState.speed);
    frc::SmartDashboard::PutNumber(description + "Angle", (double) desiredState.angle.Degrees().value());

    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.Optimize(frc::Rotation2d(units::radian_t{m_angleEncoder.GetPosition()}));

    // Set the motor speed and angle
    m_driveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage((units::angular_velocity::turns_per_second_t)
                           (desiredState.speed.value() / SwerveConstants::DriveMotorConversion.value())));
    m_turnClosedLoopController.SetReference(desiredState.angle.Radians().value(), rev::spark::SparkMax::ControlType::kPosition);
}
#pragma endregion

#pragma region GetState
/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    // Determine the module wheel velocity
    auto driveVelocity = units::meters_per_second_t {
        (double) m_driveMotor.GetVelocity().GetValue() * SwerveConstants::DriveMotorConversion.value()};

    auto anglePosition = units::radian_t{m_angleEncoder.GetPosition()};

    // Return the swerve module state
    return {driveVelocity, anglePosition};
}
#pragma endregion

#pragma region GetPosition
/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    // Determine the module wheel position
    auto drivePosition = units::meter_t{
        ((double) m_driveMotor.GetPosition().GetValue()) * SwerveConstants::DriveMotorConversion.value()};

    auto anglePosition = units::radian_t{m_angleEncoder.GetPosition()};

    // Return the swerve module position
    return {drivePosition, anglePosition};
}
#pragma endregion

#pragma region ResetDriveEncoder
// Reset the drive encoder position.
void SwerveModule::ResetDriveEncoder()
{
    m_driveMotor.SetPosition(0_tr);
}
#pragma endregion

#pragma region SetWheelAngleToForward
/// @brief Method to set the swerve wheel encoder to the forward angle.
/// @param forwardAngle The absolute angle for the forward direction.
void SwerveModule::SetWheelAngleToForward(units::angle::radian_t forwardAngle)
{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetPosition(0_tr);

    frc::SmartDashboard::PutNumber("AngleMotor Encoder Position 1", m_angleMotor.GetEncoder().GetPosition());
    
    // Set the motor angle encoder position to the forward direction
    m_angleMotor.GetEncoder().SetPosition(forwardAngle.value() - GetAbsoluteEncoderAngle().value());
    frc::SmartDashboard::PutNumber("AngleMotor Encoder Position 2", m_angleMotor.GetEncoder().GetPosition());

    m_turnClosedLoopController.SetReference(0.0, rev::spark::SparkMax::ControlType::kPosition);

    frc::SmartDashboard::PutNumber("AngleMotor Encoder Position 3", m_angleMotor.GetEncoder().GetPosition());
}
#pragma endregion

#pragma region GetAbsoluteEncoderAngle
/// @brief Method to read the absolute encode in radians.
/// @return The absolute angle value in radians.
units::angle::radian_t SwerveModule::GetAbsoluteEncoderAngle()
{
    // The GetAbsolutePosition() method returns a value from -1 to 1
    double encoderValue = (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue();

    // To convert to radians
    return encoderValue * 2.0_rad * std::numbers::pi;
}
#pragma endregion

#pragma region GetDriveEncoderRate
/// @brief Method to retrieve the drive encoder rate (velocity in meters/s).
/// @return The wheel drive encoder rate.
units::meters_per_second_t SwerveModule::GetDriveEncoderRate()
{
    // Get the motor velocity
    double rotationsPerSecond = (double) m_driveMotor.GetVelocity().GetValue();

    // Return the wheel drive encoder rate
    return (units::meters_per_second_t) rotationsPerSecond;
}
#pragma endregion
