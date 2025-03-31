#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc/XboxController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix6/TalonFX.hpp>

#include "Constants.h"

class ElevatorSubsystem : public frc2::SubsystemBase
{
public:
  enum ElevatorStates
  {
    L1 = 30,
    L2 = 41,
    L3 = 70,
    L4 = 115,
    Barge = 135,
    stow = 0,
    hold
  };

  ElevatorSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Stop();
  bool IsAtSetpoint();

  void SetElevatorState(ElevatorStates DesiredElevatorState, bool algae = false);

  // ALL WRONG - setpoint needs to be calculated
  double algaeOffset = 1;

  double setPoint ;

  int offset = 0;

private:
  ctre::phoenix6::hardware::TalonFX m_elevatorMotorLeft;
  ctre::phoenix6::hardware::TalonFX m_elevatorMotorRight;

  ctre::phoenix6::configs::TalonFXConfiguration m_elevatorConfig;

  frc::ProfiledPIDController<units::turns> profiledController;
};