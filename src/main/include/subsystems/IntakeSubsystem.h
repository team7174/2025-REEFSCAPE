#pragma once

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "subsystems/ElevatorSubsystem.h"

class IntakeSubsystem : public frc2::SubsystemBase
{
public:

  frc::XboxController *m_driveController;

  IntakeSubsystem(frc::XboxController *);
  ElevatorSubsystem *m_elevatorSubsystem;

  void Periodic() override;
  void Stop();
  // void rumbleController();

  bool FirstBeamBreakTriggered();
  bool SecondBeamBreakTriggered();
  bool CurrentSpiked();

  enum IntakeStates
  {
    coralIntake,
    algaeIntake,
    coralScore,
    algaeScore
  };

  void SetIntakeState(IntakeStates desiredCoralState);

  units::second_t intakeTimeStamp;

  IntakeStates currentState = IntakeStates::coralIntake;

  void rumbleController();
  // private means only accessable in that respective file
private:
  ctre::phoenix6::hardware::TalonFX m_coralIntakeMotor;
  ctre::phoenix6::hardware::TalonFX m_algaeIntakeMotor;
  ctre::phoenix6::hardware::TalonFX m_algaePivotMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_intakeConfig;

  frc::DigitalInput firstBeamBreak{IntakeConstants::firstIntakeBeamBreakID};
  frc::DigitalInput secondBeamBreak{IntakeConstants::secondIntakeBeamBreakID};

  double coralSpeed = 0.0;
  double algaeSpeed = 0.0;
};