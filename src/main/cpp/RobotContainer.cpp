// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
    : m_intakeSubsystem(&primaryController),
      m_elevatorSubsystem()
{
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  drivetrain.SetDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.ApplyRequest([this]() -> auto &&
                              {
                                return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed)      // Drive forward with negative Y (forward)
                                    .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)              // Drive left with negative X (left)
                                    .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                              }));

  joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto &&
                                                 { return brake; }));
  joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto &&
                                                 { return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()}); }));

  // Run SysId routines when holding back/start and X/Y.
  // Note that each routine should be run exactly once in a single log.
  (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
  (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // reset the field-centric heading on left bumper press
  joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this]
                                                  { drivetrain.SeedFieldCentric(); }));

  drivetrain.RegisterTelemetry([this](auto const &state)
                               { logger.Telemeterize(state); });

  frc2::Trigger{[this]()
                { return primaryController.GetRightTriggerAxis() > 0.5; }}
      .OnTrue( drivetrain.AutoAlign(subsystems::CommandSwerveDrivetrain::ScoringOptions::right) );
  frc2::Trigger{[this]()
                { return primaryController.GetLeftTriggerAxis() > 0.5; }}
      .OnTrue( drivetrain.AutoAlign(subsystems::CommandSwerveDrivetrain::ScoringOptions::left) );

  frc2::Trigger{[this]()
                { return secondaryController.GetLeftTriggerAxis() > 0.5 && !CoralMode; }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_intakeSubsystem.SetIntakeState(IntakeSubsystem::IntakeStates::algaeIntake); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetLeftTriggerAxis() > 0.5 && CoralMode; }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_intakeSubsystem.SetIntakeState(IntakeSubsystem::IntakeStates::coralIntake); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetBButton(); }}
      .Debounce(500_ms, frc::Debouncer::DebounceType::kBoth)
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { 
        CoralMode = !CoralMode;
        if (CoralMode) {
          m_ledSystem.UpdateSetLed([]() { return 255.0; }, []() { return 255.0; }, []() { return 255.0; }, []() { return 1.0; });
          m_ledSystem.SetColors();
        }
        else {
          m_ledSystem.UpdateSetLed([]() { return 0.0; }, []() { return 0.0; }, []() { return 255.0; }, []() { return 1.0; });
          m_ledSystem.SetColors();
        } }));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
}

RobotContainer &RobotContainer::GetInstance()
{
  static RobotContainer instance;
  return instance;
}