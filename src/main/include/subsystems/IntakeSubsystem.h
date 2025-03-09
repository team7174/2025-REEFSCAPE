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

enum CoralStates {
  hold,
  intake,
  eject,
  slow
};

enum AlgaeStates {
  hold,
  intake,
  eject,
};

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();
  ElevatorSubsystem *m_elevatorSubsystem;

  void Periodic() override;
  void Stop();
  // void rumbleController();

  bool FirstBeamBreakTriggered();
  bool SecondBeamBreakTriggered();
  bool CurrentSpiked();

  void SetCoralStates(CoralStates desiredCoralState);
  void SetAlgaeStates(AlgaeStates desiredAlgaeState);

  CoralStates currCoralState;
  AlgaeStates currAlgaeState;
  units::second_t timeOriginal;

  // private means only accessable in that respective file
 private:
  ctre::phoenix6::hardware::TalonFX m_coralIntakeMotor;
  ctre::phoenix6::hardware::TalonFX m_algaeIntakeMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_intakeConfig;

  frc::DigitalInput firstBeamBreak{IntakeConstants::firstIntakeBeamBreakID};
  frc::DigitalInput secondBeamBreak{IntakeConstants::secondIntakeBeamBreakID};

  double intakeSpeed;
  double algaeSpeed;
};