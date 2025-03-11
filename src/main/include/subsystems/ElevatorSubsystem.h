#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix6/TalonFX.hpp>

#include "Constants.h"

enum ElevatorStates {
  L1 = 5,
  L2 = 10,
  L3 = 20,
  L4 = 30,
  Barge = 140,
  stow = 0,
  hold
};

enum SuperStates {
  coral,
  algae
};

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Stop();
  bool IsAtSetpoint();

  void SetElevatorState(ElevatorStates DesiredElevatorState);

  // ALL WRONG - setpoint needs to be calculated
  double algaeOffset = 1;

  double setPoint;

 private:
  ctre::phoenix6::hardware::TalonFX m_elevatorMotorLeft;
  ctre::phoenix6::hardware::TalonFX m_elevatorMotorRight;

  ctre::phoenix6::configs::TalonFXConfiguration m_elevatorConfig;
};