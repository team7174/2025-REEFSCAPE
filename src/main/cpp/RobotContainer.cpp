// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc2/command/DeferredCommand.h>

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
                                return drive.WithVelocityX(-primaryController.GetLeftY() * MaxSpeed)      // Drive forward with negative Y (forward)
                                    .WithVelocityY(-primaryController.GetLeftX() * MaxSpeed)              // Drive left with negative X (left)
                                    .WithRotationalRate(-primaryController.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                              }));

  frc2::Trigger{[this] { return primaryController.GetAButton(); }}.WhileTrue(drivetrain.ApplyRequest([this]() -> auto &&
                                                 { return brake; }));
  frc2::Trigger{[this] { return primaryController.GetBButton(); }}.WhileTrue(drivetrain.ApplyRequest([this]() -> auto &&
                                                 { return point.WithModuleDirection(frc::Rotation2d{-primaryController.GetLeftY(), -primaryController.GetLeftX()}); }));

  // Run SysId routines when holding back/start and X/Y.
  // Note that each routine should be run exactly once in a single log.
  frc2::Trigger{[this] { return primaryController.GetBackButton() && primaryController.GetYButton(); }}.WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  frc2::Trigger{[this] { return primaryController.GetBackButton() && primaryController.GetXButton(); }}.WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
  frc2::Trigger{[this] { return primaryController.GetStartButton() && primaryController.GetYButton(); }}.WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  frc2::Trigger{[this] { return primaryController.GetStartButton() && primaryController.GetXButton(); }}.WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // reset the field-centric heading on left bumper press
  frc2::Trigger{[this] { return primaryController.GetStartButton() && primaryController.GetBackButton(); }}.OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

  drivetrain.RegisterTelemetry([this](auto const &state)
                               { logger.Telemeterize(state); });



// Assuming primaryController is your joystick or gamepad object
// Assuming primaryController is a joystick and rightTriggerAxis is an axis number
frc2::Trigger([this] { return primaryController.GetRightTriggerAxis() > 0.5; })
    .OnTrue(frc2::cmd::RunOnce([this] {
        auto command = drivetrain.AutoAlign(subsystems::CommandSwerveDrivetrain::ScoringOptions::right).Unwrap();
        if (command) {  // Make sure the command is valid
            frc2::CommandScheduler::GetInstance().Schedule(command.release());
        }
    }));



frc2::Trigger([this] { return primaryController.GetLeftTriggerAxis() > 0.5; })
    .OnTrue(frc2::cmd::RunOnce([this] {
        auto command = drivetrain.AutoAlign(subsystems::CommandSwerveDrivetrain::ScoringOptions::left).Unwrap();
        if (command) {  // Make sure the command is valid
            frc2::CommandScheduler::GetInstance().Schedule(command.release());
        }
    }));



  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(10) > 0.5 && !CoralMode; }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_intakeSubsystem.SetIntakeState(IntakeSubsystem::IntakeStates::algaeIntake); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(10) > 0.5 && CoralMode; }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_intakeSubsystem.SetIntakeState(IntakeSubsystem::IntakeStates::coralIntake);
                                   m_elevatorSubsystem.SetElevatorState(ElevatorSubsystem::ElevatorStates::stow); }));
  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(7) > 0.5 && CoralMode; }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_intakeSubsystem.SetIntakeState(IntakeSubsystem::IntakeStates::coralScore); }));
  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(7) > 0.5 && !CoralMode; }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_intakeSubsystem.SetIntakeState(IntakeSubsystem::IntakeStates::algaeScore); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(1); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_elevatorSubsystem.SetElevatorState(ElevatorSubsystem::ElevatorStates::L4); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(2); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_elevatorSubsystem.SetElevatorState(ElevatorSubsystem::ElevatorStates::L3, !CoralMode); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(3); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_elevatorSubsystem.SetElevatorState(ElevatorSubsystem::ElevatorStates::L2, !CoralMode); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(4); }}
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { m_elevatorSubsystem.SetElevatorState(ElevatorSubsystem::ElevatorStates::L1); }));

  frc2::Trigger{[this]()
                { return secondaryController.GetRawButton(8); }}
      .Debounce(50_ms, frc::Debouncer::DebounceType::kBoth)
      .OnTrue(frc2::cmd::RunOnce([this]
                                 { 
        CoralMode = !CoralMode;
        frc::SmartDashboard::PutBoolean("Coral Mode", CoralMode);
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