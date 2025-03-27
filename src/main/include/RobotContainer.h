// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"

#include "frc/XboxController.h"
#include "frc/Joystick.h"

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/CANdleSystem.h"
#include "subsystems/ClimbSubsystem.h"

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = 3_mps; //TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.5_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

    IntakeSubsystem m_intakeSubsystem;
    ElevatorSubsystem m_elevatorSubsystem;
    ClimbSubsystem m_climbSubsystem;
    CANdleSystem m_ledSystem;


    bool CoralMode = true;

    static RobotContainer& GetInstance();

private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> autoChooser;

public:
    RobotContainer();

    frc2::Command *GetAutonomousCommand();

private:
    void ConfigureBindings();

    frc::XboxController primaryController{0};
    frc::Joystick secondaryController{1};

};
