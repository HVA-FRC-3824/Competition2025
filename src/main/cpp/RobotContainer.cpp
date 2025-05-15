#include "RobotContainer.h"

// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = NULL;

#pragma region GetInstance
/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer *RobotContainer::GetInstance()
{
    // Detrermine if the class has already been instantiated
    if (m_robotContainer == NULL)
    {
        // Instantiate the class
        m_robotContainer = new RobotContainer();
    }

    // Return the class pointer
    return m_robotContainer;
}
#pragma endregion

#pragma region RobotContainer
/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer()
{
    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    // frc::SmartDashboard::PutData("Chassis: Time",           new ChassisDriveTime(2_s, 0.5_mps,                                                   &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: X 50 Inches",    new ChassisDrivePose(2.0_mps,  50_in,   0_in,  0_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: X -50 Inches",   new ChassisDrivePose(2.0_mps, -50_in,   0_in,  0_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: X 100 Inches",   new ChassisDrivePose(2.0_mps, 100_in,   0_in,  0_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: Y 50 Inches",    new ChassisDrivePose(2.0_mps,   0_in,  50_in,  0_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: Y 100 Inches",   new ChassisDrivePose(2.0_mps,   0_in, 100_in,  0_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: XY 50 Inches",   new ChassisDrivePose(2.0_mps,  50_in,  50_in,  0_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: XY 100 Inches",  new ChassisDrivePose(2.0_mps, 100_in, 100_in,  0_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: XY 100 Turn",    new ChassisDrivePose(2.0_mps, 100_in, 100_in, 45_deg,      10_s,                     &m_drivetrain));
    // frc::SmartDashboard::PutData("Chassis: Serpentine",     new ChassisDriveSerpentine(1.0_mps,                      10_s,                       &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: AprilTag ",      new ChassisDriveToAprilTag([this] { return GetChassisDriveToAprilTagParameters(); }, &m_drivetrain));

    // frc::SmartDashboard::PutData("Elevator Jog Up",          new frc2::InstantCommand([this] { m_gripper.SetElevatorOffset( ElevatorConstants::HeightOffset); }));
    // frc::SmartDashboard::PutData("Elevator Jog Down",        new frc2::InstantCommand([this] { m_gripper.SetElevatorOffset(-ElevatorConstants::HeightOffset); }));

    // frc::SmartDashboard::PutData("Arm Jog Positive",         new frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset( ArmConstants::AngleOffset);}));
    // frc::SmartDashboard::PutData("Arm Jog Negative",         new frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset(-ArmConstants::AngleOffset);}));

    // frc::SmartDashboard::PutData("Wrist Jog Positive",       new frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset( WristConstants::AngleOffset);}));
    // frc::SmartDashboard::PutData("Wrist Jog Negative",       new frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset(-WristConstants::AngleOffset);}));

    // frc::SmartDashboard::PutData("Coral: Ground",            new GripperPose(GripperPoseEnum::CoralGround,    &m_gripper));
    // frc::SmartDashboard::PutData("Coral: Station",           new GripperPose(GripperPoseEnum::CoralStation,   &m_gripper));
    // frc::SmartDashboard::PutData("Coral: L1",                new GripperPose(GripperPoseEnum::CoralL1,        &m_gripper));
    // frc::SmartDashboard::PutData("Coral: L2",                new GripperPose(GripperPoseEnum::CoralL2,        &m_gripper));
    // frc::SmartDashboard::PutData("Coral: L3",                new GripperPose(GripperPoseEnum::CoralL3,        &m_gripper));
    // frc::SmartDashboard::PutData("Coral: L4",                new GripperPose(GripperPoseEnum::CoralL4,        &m_gripper));

    // frc::SmartDashboard::PutData("Algae: Ground",            new GripperPose(GripperPoseEnum::AlgaeGround,    &m_gripper));
    // frc::SmartDashboard::PutData("Algae: Coral",             new GripperPose(GripperPoseEnum::AlgaeOnCoral,   &m_gripper));
    // frc::SmartDashboard::PutData("Algae: Low",               new GripperPose(GripperPoseEnum::AlgaeLow,       &m_gripper));
    // frc::SmartDashboard::PutData("Algae: High",              new GripperPose(GripperPoseEnum::AlgaeHigh,      &m_gripper));
    // frc::SmartDashboard::PutData("Algae: Processor",         new GripperPose(GripperPoseEnum::AlgaeProcessor, &m_gripper));
    // frc::SmartDashboard::PutData("Algae: Barge",             new GripperPose(GripperPoseEnum::AlgaeBarge,     &m_gripper));

    //frc::SmartDashboard::PutData("Gripper: Activate",        new GripperActivate(&m_gripper));

    // Configure the autonomous command chooser
    m_autonomousChooser.SetDefaultOption("Do Nothing",       new AutonomousDoNothing());
    m_autonomousChooser.AddOption("Drive Forward",           new ChassisDrivePose(1.0_mps, 1_m, 0_m, 0_deg, 10_s, &m_drivetrain));

    m_autonomousChooser.AddOption("Place Coral L1",          new AutonomousOneCoral(GripperPoseEnum::CoralAutonomousL1,
                                                                    [this] { return GetAutonomousOneCoralParameters(-5_in, 0_in); },
                                                                    &m_drivetrain, &m_gripper));
    // m_autonomousChooser.AddOption("Place Coral L2",          new AutonomousOneCoral(GripperPoseEnum::CoralL2,
    //                                                                 [this] { return GetAutonomousOneCoralParameters(0_in, 4_in);  },
    //                                                                 &m_drivetrain, &m_gripper));
    // m_autonomousChooser.AddOption("Place Coral L3",          new AutonomousOneCoral(GripperPoseEnum::CoralL3,
    //                                                                 [this] { return GetAutonomousOneCoralParameters(0_in, 4_in);  },
    //                                                                 &m_drivetrain, &m_gripper));
    m_autonomousChooser.AddOption("Place Coral L4",          new AutonomousOneCoral(GripperPoseEnum::CoralL4,
                                                                    [this] { return GetAutonomousOneCoralParameters(2_in, 4_in);  },
                                                                    &m_drivetrain, &m_gripper));

    m_autonomousChooser.AddOption("Place Coral L1 AprilTag", new AutonomousOneCoralAprilTag(GripperPoseEnum::CoralAutonomousL1,
                                                                    [this] { return GetStartPosition();                           },
                                                                    [this] { return GetAutonomousOneCoralAprilTagParameters();    },
                                                                    [this] { return GetChassisDriveToAprilTagParameters();        },
                                                                    &m_drivetrain, &m_gripper, &m_climb));
    m_autonomousChooser.AddOption("Place Coral L4 AprilTag", new AutonomousOneCoralAprilTag(GripperPoseEnum::CoralL4,
                                                                    [this] { return GetStartPosition();                           },
                                                                    [this] { return GetAutonomousOneCoralAprilTagParameters();    },
                                                                    [this] { return GetChassisDriveToAprilTagParameters();        },
                                                                    &m_drivetrain, &m_gripper, &m_climb));

    // m_autonomousChooser.AddOption("Place Coral and Algae",   new AutonomousCoralAndAlgae(GripperPoseEnum::CoralL4,
    //                                                                 [this] { return GetStartPosition();                           },
    //                                                                 [this] { return GetAutonomousOneCoralAprilTagParameters();    },
    //                                                                 [this] { return GetChassisDriveToAprilTagParameters();        },
    //                                                                 &m_drivetrain, &m_gripper));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autonomousChooser);

    m_startingPositionChooser.SetDefaultOption("Middle", "M");
    m_startingPositionChooser.AddOption("Left",          "L");
    m_startingPositionChooser.AddOption("Right",         "R");

    frc::SmartDashboard::PutData("Start Position", &m_startingPositionChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive([this] { return Forward(); },
                                                [this] { return Strafe();  },
                                                [this] { return Angle();   },
                                                &m_drivetrain));

    // Set the LED default command
    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));

    // Set the swerve wheels to zero
    SetSwerveWheelAnglesToZero();

    // Start capturing video from the USB camera
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    // Set the resolution and frame rate of the camera
    camera.SetResolution(640, 480); // Set resolution to 640x480
    camera.SetFPS(30);             // Set frame rate to 30 FPS

    // Access the Limelight feed
    // m_limelightFeed = frc::CameraServer::AddSwitchedCamera("Limelight");

    // Set the Limelight feed source to a valid video source
    // m_limelightFeed.SetSource(frc::CameraServer::GetVideo().GetSource());

    // // Create a VideoSink to control the stream
    // m_server = frc::CameraServer::GetServer();

    // // Set the initial camera source to the Limelight camera
    // m_server.SetSource(m_limelightFeed.GetSource());

    //frc::SmartDashboard::PutString("Active Camera", "USB Camera");
}
#pragma endregion

#pragma region ConfigureButtonBindings
/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    // Configure the driver controls
    ConfigureDriverControls();

    // Configure the operator controls
    ConfigureCoralPoseControls();
    ConfigureAlgaePoseControls();
    ConfigureGripperControls();
    ConfigureClimberControls();
}
#pragma endregion

#pragma region ConfigureDriverControls
/// @brief Method to bind the driver joystick controls to the robot commands.
void RobotContainer::ConfigureDriverControls()
{
    // Drive to position using the AprilTag
    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::HandleSide)
        .WhileTrue(new ChassisDriveToAprilTag([this] { return GetChassisDriveToAprilTagParameters(); }, &m_drivetrain));

    // Use the trigger to activate the operation (Scores/Intakes Algae/Coral)
    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::HandleTrigger)
        .WhileTrue(new GripperActivate(&m_gripper));

    // Reset the gyro angle
    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::HandleUpperLeft)
        .OnTrue(new frc2::InstantCommand([this] { m_drivetrain.ZeroHeading(); }, {&m_drivetrain}));

    // // Switch between camera feeds
    // frc2::JoystickButton(&m_driverController, ConstantsExtreme3D::HandleUpperRight).OnTrue(new frc2::InstantCommand([this]
    // {
    //    // Toggle the active camera flag
    //    m_usbCameraActive = !m_usbCameraActive;

    //    // Update the video source based on the active camera
    //    if (m_usbCameraActive)
    //    {
    //        m_server.SetSource(m_usbCamera);
    //        frc::SmartDashboard::PutString("Active Camera", "USB Camera");
    //    }
    //    else
    //    {
    //        m_server.SetSource(m_limelightFeed.GetSource());
    //        frc::SmartDashboard::PutString("Active Camera", "Limelight");
    //    }
    // }));

    // Reset the gyro angle
    // frc2::JoystickButton (&m_driverController, Extreme3DConstants::HandleUpperRight)
        // .OnTrue(new frc2::InstantCommand([this] { m_drivetrain.ZeroHeadingReverse(); }, {&m_drivetrain}));

    // Set field centricity on
    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::HandleLowerLeft)
        .OnTrue(new frc2::InstantCommand([this] { m_drivetrain.SetFieldCentricity(true); }, {&m_drivetrain}));

    // Set field centricity off
    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::HandleLowerRight)
        .OnTrue(new frc2::InstantCommand([this] { m_drivetrain.SetFieldCentricity(false); }, {&m_drivetrain}));

    // Toggle X mode
    frc2::JoystickButton (&m_driverController, frc::XboxController::Button::kX)
        .WhileTrue(new frc2::RunCommand([this] { m_drivetrain.SetX(); }, {&m_drivetrain}));

    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::Handle12)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(ElevatorConstants::HeightOffset);}));

    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::Handle11)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(-ElevatorConstants::HeightOffset);}));

    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::Handle10)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetArmAngleOffset(-ArmConstants::AngleOffset);}));

    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::Handle9)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetArmAngleOffset(ArmConstants::AngleOffset);}));

    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::Handle8)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetWristAngleOffset(WristConstants::AngleOffset);}));

    frc2::JoystickButton (&m_driverController, ConstantsExtreme3D::Handle7)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetWristAngleOffset(-WristConstants::AngleOffset);}));
}
#pragma endregion

#pragma region ConfigureGripperControls
/// @brief Method to bind the operator control panel gripper controls.
void RobotContainer::ConfigureGripperControls()
{
    frc2::JoystickButton (&m_operatorController, ConstantsControlPanel::OperatorWheels)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper
        .SetGripperWheelsVoltage([this] { return PotentiometerWheelVoltage(); }); }, {&m_gripper}));

    frc2::JoystickButton (&m_operatorController, ConstantsControlPanel::Home)
        .OnTrue(GripperPose(GripperPoseEnum::Home, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Manually offsets elevator upwards
    frc2::JoystickButton (&m_operatorController, ConstantsControlPanel::ElevatorUp)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(ElevatorConstants::HeightOffset);}));

    // Manually offsets elevator downwards
    frc2::JoystickButton (&m_operatorController, ConstantsControlPanel::ElevatorDown)
        .WhileTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(-ElevatorConstants::HeightOffset);}));
}
#pragma endregion

#pragma region ConfigureCoralPoseControls
/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: L1Score then activate).
void RobotContainer::ConfigureCoralPoseControls()
{
    // Define an array of button mappings for coral poses
    struct CoralPoseMapping
    {
        int             button;
        GripperPoseEnum pose;
    };

    CoralPoseMapping coralPoses[] =
    {
        {ConstantsControlPanel::CoralGnd, GripperPoseEnum::CoralGround},
        {ConstantsControlPanel::CoralStn, GripperPoseEnum::CoralStation},
        {ConstantsControlPanel::CoralL1,  GripperPoseEnum::CoralL1},
        {ConstantsControlPanel::CoralL2,  GripperPoseEnum::CoralL2},
        {ConstantsControlPanel::CoralL3,  GripperPoseEnum::CoralL3},
        {ConstantsControlPanel::CoralL4,  GripperPoseEnum::CoralL4}
    };

    // Iterate through the array and bind the buttons to the corresponding poses
    for (const auto& mapping : coralPoses)
    {
        frc2::JoystickButton (&m_operatorController, mapping.button)
            .OnTrue(GripperPose(mapping.pose, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}
#pragma endregion

#pragma region ConfigureAlgaePoseControls
/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: AlgaeBarge then activate).
void RobotContainer::ConfigureAlgaePoseControls()
{
    // Define an array of button mappings for algae poses
    struct AlgaePoseMapping
    {
        int             button;
        GripperPoseEnum pose;
    };

    AlgaePoseMapping algaePoses[] =
    {
        {ConstantsControlPanel::AlgaeGnd,       GripperPoseEnum::AlgaeGround},
        {ConstantsControlPanel::AlgaeCoral,     GripperPoseEnum::AlgaeOnCoral},
        {ConstantsControlPanel::AlgaeLow,       GripperPoseEnum::AlgaeLow},
        {ConstantsControlPanel::AlgaeHigh,      GripperPoseEnum::AlgaeHigh},
        {ConstantsControlPanel::AlgaeProcessor, GripperPoseEnum::AlgaeProcessor},
        {ConstantsControlPanel::AlgaeBarge,     GripperPoseEnum::AlgaeBarge}
    };

    // Iterate through the array and bind the buttons to the corresponding poses
    for (const auto& mapping : algaePoses)
    {
        frc2::JoystickButton (&m_operatorController, mapping.button)
            .OnTrue(GripperPose(mapping.pose, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}
#pragma endregion

#pragma region ConfigureClimberControls
/// @brief Method to bind the operator control panel climb controls.
void RobotContainer::ConfigureClimberControls()
{
    // Manually offsets climb upwards
    frc2::JoystickButton (&m_operatorController, ConstantsControlPanel::ClimbUp)
        .WhileTrue(new frc2::RunCommand([this] { m_climb.SetVoltage(ClimbConstants::ClimbVoltage); }, {&m_climb}))
        .OnFalse(new frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}));

    // Manually offsets climb downwards
    frc2::JoystickButton (&m_operatorController, ConstantsControlPanel::ClimbDown)
        .WhileTrue(new frc2::RunCommand([this] { m_climb.SetVoltage(-ClimbConstants::ClimbVoltage); }, {&m_climb}))
        .OnFalse(new frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}));
}
#pragma endregion

#pragma region GetDriverController
/// @brief Method to return a pointer to the driver joystick.
/// @return Pointer to the driver joystick.
frc::Joystick *RobotContainer::GetDriverController()
{
    // Return the pointer to the driver joystick
    return &m_driverController;
}
#pragma endregion

#pragma region GetOperatorController
/// @brief Method to return a pointer to the controller joystick.
/// @return Pointer to the controller joystick.
frc::XboxController *RobotContainer::GetOperatorController()
{
    // Return the pointer to the operator joystick
    return &m_operatorController;
}
#pragma endregion

#pragma region GetAutonomousCommand
/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // The selected command will be run in autonomous
    return m_autonomousChooser.GetSelected();
}
#pragma endregion

#pragma region SetSwerveWheelAnglesToZero
 /// @brief Method to set the swerve wheels to starting position based on the absolute encoder.
 void RobotContainer::SetSwerveWheelAnglesToZero()
 {
    // Execute the command
    m_swerveWheelAnglesToZero->Execute();
 }
#pragma endregion

#pragma region Forward
/// @brief Method to return the forward joystick value.
/// @return The forward joystick meters per second value.
units::meters_per_second_t RobotContainer::Forward()
{
    // Get the forward joystick setting
    double joystickForward = GetDriverController()->GetRawAxis(ConstantsController::JoystickForwardIndex);

    // Modify the joystick value by the "throttle" setting
    joystickForward *= GetThrottleRange();

    // Use exponential function to calculate the forward value for better slow speed control
    joystickForward = GetExponentialValue(joystickForward, ConstantsController::ExponentForward);

    // Return the x speed
    return -m_xspeedLimiter.Calculate(frc::ApplyDeadband(joystickForward, ConstantsController::JoystickDeadZone)) * DrivetrainConstants::MaxSpeed;
}
#pragma endregion

#pragma region Strafe
/// @brief Method to return the strafe joystick value.
/// @return The strafe joystick meters per second value.
units::meters_per_second_t RobotContainer::Strafe()
{
    // Get the strafe joystick setting
    double joystickStrafe = GetDriverController()->GetRawAxis(ConstantsController::JoystickStrafeIndex);

    // Modify the joystick value by the "throttle" setting
    joystickStrafe *= GetThrottleRange();

    // Use exponential function to calculate the forward value for better slow speed control
    joystickStrafe = GetExponentialValue(joystickStrafe, ConstantsController::ExponentStrafe);

    // Return the y speed
    return -m_yspeedLimiter.Calculate(frc::ApplyDeadband(joystickStrafe, ConstantsController::JoystickDeadZone)) * DrivetrainConstants::MaxSpeed;
}
#pragma endregion

#pragma region Angle
/// @brief Method to return the angle joystick value.
/// @return The angle joystick value.
units::radians_per_second_t RobotContainer::Angle()
{
    // Get the angle joystick setting
    double joystickAngle = GetDriverController()->GetRawAxis(ConstantsController::JoystickAngleIndex);

    // Apply deadband first
    double deadbandedAngle = frc::ApplyDeadband(joystickAngle, ConstantsController::JoystickRotateDeadZone);


    // Use exponential function to calculate the angle value for better slow speed control
    double exponentialAngle = GetExponentialValue(deadbandedAngle, ConstantsController::ExponentAngle);

    // Apply smoothing between frames to reduce jerky movement (inline implementation)
    // Smoothing factor: 0.0-1.0 (higher = more smoothing, 0.3 is a good starting point)
    constexpr double kSmoothingFactor   = 0.3;
    static double    previousAngleInput = 0.0; // Static variable persists between function calls

    // Calculate smoothed value using previous output and current input
    double smoothedAngle = kSmoothingFactor * previousAngleInput + (1.0 - kSmoothingFactor) * exponentialAngle;
    previousAngleInput   = smoothedAngle; // Store for next cycle

    // Modify the joystick value by the "throttle" setting
    smoothedAngle *= GetThrottleRange();

    // Return the rotation speed with rate limiter applied
    return units::radians_per_second_t(-m_rotLimiter.Calculate(smoothedAngle) * DrivetrainConstants::MaxAngularSpeed * 0.5);
}
#pragma endregion

#pragma region GetThrottleRange
/// @brief Method to convert the throttle range to a value between ThrottleMinimum and 1.0.
/// @return The throttle value.
double RobotContainer::GetThrottleRange()
{
    auto throttle = -GetDriverController()->GetRawAxis(ConstantsController::JoystickThrottleIndex);

    // Convert the throttle value from -1.0 to 1.0 to 0.0 to 1.0
    throttle = (throttle + 1.0) / 2.0;

    // Set the throttle to the minimum to maximum range
    throttle = (1 - ConstantsController::ThrottleMinimum) * throttle + ConstantsController::ThrottleMinimum;

    // Return the throttle value
    return throttle;
}
#pragma endregion

#pragma region ExponentialValue
/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resultant exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = (joystickValue < 0.0) ? -1 : 1;
    double absValue  = std::abs(joystickValue);
    double output    = std::pow(absValue, exponent) * direction;

    // Ensure the range of the output
    if (output < -1.0) output = -1.0;
    if (output > 1.0)  output = 1.0;

    // Return the output value
    return output;
}
#pragma endregion

#pragma region PotentiometerWheelVoltage
/// @brief Method to get the potentiometer wheel voltage.
/// @return The potentiometer wheel voltage.
GripperWheelState RobotContainer::PotentiometerWheelVoltage()
{
    // Read the wheel voltage potentiometer
    auto potentiometer = -(m_operatorController.GetRawAxis(ConstantsControlPanel::GripperMotor) - GripperConstants::MeanAnalogInput);

    // Apply a deadband to the potentiometer
    if (potentiometer < GripperConstants::GripperWheelDeadZone && potentiometer > -GripperConstants::GripperWheelDeadZone)
        potentiometer = 0.0;

    frc::SmartDashboard::PutNumber("Potentiometer", potentiometer);

    // Convert to a voltage
    auto voltage = units::voltage::volt_t{potentiometer * GripperConstants::AnalogConversion};

    // Determine if both wheels are active
    bool bothWheels = !m_operatorController.GetRawButton(ConstantsControlPanel::BothWheelsActive);

    // Return the gripper wheel state
    GripperWheelState gripperWheelState;
    gripperWheelState.bothWheels = bothWheels;
    gripperWheelState.voltage    = voltage;

    // Return the gripper wheel state
    return gripperWheelState;
}
#pragma endregion

#pragma region GetChassisPose
/// @brief Method to get the chassis Pose.
/// @return The chassis Pose.
frc::Pose2d RobotContainer::GetChassisPose()
{
    // Return the chassis pose
    return m_drivetrain.GetPose();
}
#pragma endregion

#pragma region ReverseChassisGryo
/// @brief Method to reverse the Chassis heading to account for field centric drive with the robot facing the driver.
void RobotContainer::ReverseChassisGryo()
{
    // Reverse the chassis gyro
    m_drivetrain.ReverseHeading();
}
#pragma endregion

#pragma region GetGripper
/// @brief Method to return a pointer to the gripper subsystem.
/// @return Pointer to the gripper subsystem.
Gripper *RobotContainer::GetGripper()
{
    // Return the pointer to the gripper
    return &m_gripper;
}
#pragma endregion

#pragma region GetStartPosition
/// @brief Method to get the starting position for the robot.
/// @return String representing the starting position.
std::string RobotContainer::GetStartPosition()
{
    // Return the selected starting position
    return m_startingPositionChooser.GetSelected();;
}
#pragma endregion

#pragma region GetAutonomousOneCoralParameters
/// @brief Method to get the autonomous one coral parameters.
/// @return The autonomous one coral parameters.
ChassDrivePoseParameters RobotContainer::GetAutonomousOneCoralParameters(units::length::inch_t distanceXOffset, units::length::inch_t distanceYOffset)
{
    ChassDrivePoseParameters parameters;

    // Set the common pose parameters
    parameters.Speed       = ConstantsChassisPoseAutonomous::AutonomousSpeed;
    parameters.TimeoutTime = ConstantsChassisPoseAutonomous::AutonomousTimeOut;

    std::string startPosition = "Unknown";

    // Get the starting position
    startPosition = GetStartPosition();

    // Determine the starting position based on the selected string position ("L", "M", "R")
    if (startPosition.compare("L") == 0)
    {
        // Set the left coral position
        startPosition        = "L";
        parameters.DistanceX = ConstantsChassisPoseAutonomous::OneCoralLeftXDistance;
        parameters.DistanceY = ConstantsChassisPoseAutonomous::OneCoralLeftYDistance + distanceYOffset;
        parameters.Angle     = ConstantsChassisPoseAutonomous::OneCoralLeftAngleChange;

    }
    else if (startPosition.compare("M") == 0)
    {
        // Set the middle coral position
        startPosition         = "M";
        parameters.DistanceX  = ConstantsChassisPoseAutonomous::OneCoralMiddleXDistance + distanceXOffset;
        parameters.DistanceY  = ConstantsChassisPoseAutonomous::OneCoralMiddleYDistance;
        parameters.Angle      = ConstantsChassisPoseAutonomous::OneCoralAngleChange;
    }
    else if (startPosition.compare("R") == 0)
    {
        // Set the right coral position
        startPosition        = "R";
        parameters.DistanceX = ConstantsChassisPoseAutonomous::OneCoralRightXDistance;
        parameters.DistanceY = ConstantsChassisPoseAutonomous::OneCoralRightYDistance - distanceYOffset;
        parameters.Angle     = ConstantsChassisPoseAutonomous::OneCoralRightAngleChange;
    }

    // frc::SmartDashboard::PutString("Coral Start Position", startPosition);
    // frc::SmartDashboard::PutNumber("Coral X Distance",     parameters.DistanceX.to<double>() * 39.3701);
    // frc::SmartDashboard::PutNumber("Coral Y Distance",     parameters.DistanceY.to<double>() * 39.3701);
    // frc::SmartDashboard::PutNumber("Coral Angle Change",   parameters.Angle.to<double>());

    // Return the parameters
    return parameters;
}
#pragma endregion

#pragma region GetAutonomousOneCoralAprilTagParameters
/// @brief Method to get the autonomous one coral parameters.
/// @return The autonomous one coral parameters.
ChassDrivePoseParameters RobotContainer::GetAutonomousOneCoralAprilTagParameters()
{
    ChassDrivePoseParameters parameters;

    // Set the common pose parameters
    parameters.Speed       = ConstantsChassisPoseAutonomous::AutonomousSpeed;
    parameters.TimeoutTime = ConstantsChassisPoseAutonomous::AutonomousTimeOut;

    std::string startPosition = "Unknown";

    // Get the starting position
    startPosition = GetStartPosition();

    // Determine the starting position based on the selected string position ("L", "M", "R")
    if (startPosition.compare("L") == 0)
    {
        // Set the left coral position
        startPosition        = "L";
        parameters.DistanceX = ConstantsChassisPoseAutonomous::OneCoralLeftXDistanceAprilTag;
        parameters.DistanceY = ConstantsChassisPoseAutonomous::OneCoralLeftYDistanceAprilTag;
        parameters.Angle     = ConstantsChassisPoseAutonomous::OneCoralLeftAngleChangeAprilTag;

    }
    else if (startPosition.compare("M") == 0)
    {
        // Set the middle coral position
        startPosition         = "M";
        parameters.DistanceX  = ConstantsChassisPoseAutonomous::OneCoralMiddleXDistanceAprilTag;
        parameters.DistanceY  = ConstantsChassisPoseAutonomous::OneCoralMiddleYDistanceAprilTag;
        parameters.Angle      = ConstantsChassisPoseAutonomous::OneCoralAngleChangeAprilTag;
    }
    else if (startPosition.compare("R") == 0)
    {
        // Set the right coral position
        startPosition        = "R";
        parameters.DistanceX = ConstantsChassisPoseAutonomous::OneCoralRightXDistanceAprilTag;
        parameters.DistanceY = ConstantsChassisPoseAutonomous::OneCoralRightYDistanceAprilTag;
        parameters.Angle     = ConstantsChassisPoseAutonomous::OneCoralRightAngleChangeAprilTag;
    }

    // frc::SmartDashboard::PutString("Coral Start Position", startPosition);
    // frc::SmartDashboard::PutNumber("Coral X Distance",     parameters.DistanceX.to<double>());
    // frc::SmartDashboard::PutNumber("Coral Y Distance",     parameters.DistanceY.to<double>());
    // frc::SmartDashboard::PutNumber("Coral Angle Change",   parameters.Angle.to<double>());

    // Return the parameters
    return parameters;
}
#pragma endregion

#pragma region GetAutonomousTwoCoralAprilTagParameters
ChassDrivePoseParameters RobotContainer::GetAutonomousTwoCoralAprilTagParameters()
{
    ChassDrivePoseParameters parameters;

    parameters.Speed = ConstantsChassisPoseAutonomous::AutonomousSpeed;
    parameters.TimeoutTime = ConstantsChassisPoseAutonomous::AutonomousTimeOut;
    //paremeters.DistanceX = ConstantsChassisPoseAutonomous::OneCoralLeftXDistanceAprilTag;
}
#pragma endregion

//#define READ_FROM_SMARTDASHBOARD

#pragma region GetChassisDriveToAprilTagParameters
/// @brief  Method to return the parameters for the ChassisDriveToAprilTag command.
/// @return The parameters for the ChassisDriveToAprilTag command.
///
///     bool                       ValidPose;        Must be a case in the switch statement
///     bool                       ReefRightSide;    Read from the control panel
///
///     units::meters_per_second_t Speed;             Speed is set for all valid poses
///     units::meter_t             DistanceOffsetX;   --+
///     units::meter_t             DistanceOffsetY;     | Set in the case statement
///     units::degree_t            AngleOffset;       --+
///     units::time::second_t      TimeoutTime;       Time-out time is the same for all valid poses
ChassDriveAprilTagParameters RobotContainer::GetChassisDriveToAprilTagParameters()
{
    ChassDriveAprilTagParameters parameters;

    // Sets the field centricity to true after getting to the position.
    parameters.FieldCentricity = true;

    // Assume the pose is valid
    parameters.ValidPose = true;

    // Set the remaining parameters that are not set in the case statement
    parameters.PoseParameters.Speed       = ConstantsChassisAprilTagToPose::ChassisSpeed;        // Speed of the chassis
    parameters.PoseParameters.TimeoutTime = ConstantsChassisAprilTagToPose::TimeoutTime;         // Time-out time for the command

    // Determine the side of the reef
    m_operatorController.GetRawButton(ConstantsControlPanel::CoralSideSelect) ? parameters.ReefRightSide = true : parameters.ReefRightSide = false;

#ifdef READ_FROM_SMARTDASHBOARD
    parameters.PoseParameters.DistanceX = units::inch_t   {frc::SmartDashboard::GetNumber ("AprilTag: DistanceOffsetX", 0.0)};
    parameters.PoseParameters.DistanceY = units::inch_t   {frc::SmartDashboard::GetNumber ("AprilTag: DistanceOffsetY", 0.0)};
    parameters.PoseParameters.Angle     = units::degree_t {frc::SmartDashboard::GetNumber ("AprilTag: AngleOffset",     0.0)};
#else
    // Get the pose from the gripper
    auto gripperPose = m_gripper.GetPose();

    // Determine the pose to drive to
    switch (gripperPose)
    {
        case GripperPoseEnum::CoralStation:  // Drive to the coral station for feeding coral
        {
            parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::CoralStationDistanceOffsetX;
            parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::CoralStationDistanceOffsetY;
            parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::CoralStationAngleOffset;
            break;
        }

        case GripperPoseEnum::CoralL1:  // Drive to the coral reef for placing on L1
        case GripperPoseEnum::CoralAutonomousL1:
        {
            // Determine the side of the reef
            if (parameters.ReefRightSide)
            {
                parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::CoralL1ReefLeftDistanceOffsetX;
                parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::CoralL1ReefLeftDistanceOffsetY;
                parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::CoralL1ReefLeftAngleOffset;
            }
            else
            {
                parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::CoralL1ReefRightDistanceOffsetX;
                parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::CoralL1ReefRightDistanceOffsetY;
                parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::CoralL1ReefRightAngleOffset;
            }
            break;
        }

        case GripperPoseEnum::CoralL2:  // Drive to the coral reef for placing on L2 or L3
        case GripperPoseEnum::CoralL3:
        {
            // Determine the side of the reef
            if (parameters.ReefRightSide)
            {
                parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::CoralL23ReefLeftDistanceOffsetX;
                parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::CoralL23ReefLeftDistanceOffsetY;
                parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::CoralL23ReefLeftAngleOffset;
            }
            else
            {
                parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::CoralL23ReefRightDistanceOffsetX;
                parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::CoralL23ReefRightDistanceOffsetY;
                parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::CoralL23ReefRightAngleOffset;
            }
            break;
        }

        case GripperPoseEnum::CoralL4:  // Drive to the coral reef for placing on L4
        {
            // Determine the side of the reef
            if (parameters.ReefRightSide)
            {
                parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::CoralL4ReefLeftDistanceOffsetX;
                parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::CoralL4ReefLeftDistanceOffsetY;
                parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::CoralL4ReefLeftAngleOffset;
            }
            else
            {
                parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::CoralL4ReefRightDistanceOffsetX;
                parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::CoralL4ReefRightDistanceOffsetY;
                parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::CoralL4ReefRightAngleOffset;
            }
            break;
        }

        case GripperPoseEnum::AlgaeLow:   // Drive to the reef for extracting algae
        case GripperPoseEnum::AlgaeHigh:
        {
            parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::AlgaeReefDistanceOffsetX;
            parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::AlgaeReefDistanceOffsetY;
            parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::AlgaeReefAngleOffset;
            break;
        }

        case GripperPoseEnum::AlgaeProcessor:  // Drive to the algae processor for processing algae
        {
            parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::AlgaeProcessorDistanceOffsetX;
            parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::AlgaeProcessorDistanceOffsetY;
            parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::AlgaeProcessorAngleOffset;
            break;
        }

        case GripperPoseEnum::AlgaeBarge:  // Drive to the algae barge for depositing algae on the barge
        {
            parameters.PoseParameters.DistanceX = ConstantsChassisAprilTagToPose::AlgaelBargeDistanceOffsetX;
            parameters.PoseParameters.DistanceY = ConstantsChassisAprilTagToPose::AlgaelBargeDistanceOffsetY;
            parameters.PoseParameters.Angle     = ConstantsChassisAprilTagToPose::AlgaelBargeAngleOffset;
            break;
        }

        default:  // Not a valid pose to drive to an april tag
        {
            parameters.ValidPose = false;
            break;
        }
    }
#endif

    frc::SmartDashboard::PutNumber("AprilTag: ValidPose",       parameters.ValidPose);
    frc::SmartDashboard::PutNumber("AprilTag: ReefRightSide",   parameters.ReefRightSide);
    frc::SmartDashboard::PutNumber("AprilTag: Speed",           parameters.PoseParameters.Speed.to<double>());
    frc::SmartDashboard::PutNumber("AprilTag: DistanceOffsetX", parameters.PoseParameters.DistanceX.to<double>() * 39.3701);
    frc::SmartDashboard::PutNumber("AprilTag: DistanceOffsetY", parameters.PoseParameters.DistanceY.to<double>() * 39.3701);
    frc::SmartDashboard::PutNumber("AprilTag: AngleOffset",     parameters.PoseParameters.Angle.to<double>());
    frc::SmartDashboard::PutNumber("AprilTag: TimeoutTime",     parameters.PoseParameters.TimeoutTime.to<double>());

    // Return the parameters
    return parameters;
}
#pragma endregion

#pragma region GetPowerDistribution
/// @brief Method to return a pointer to the power distribution panel.
frc::PowerDistribution *RobotContainer::GetPowerDistribution()
{
    // Return the pointer to the power distribution panel
    return &m_powerDistribution;
}
#pragma endregion
