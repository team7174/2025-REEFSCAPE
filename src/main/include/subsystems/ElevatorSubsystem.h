#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix6/TalonFX.hpp>

#include "Constants.h"

enum ElevatorStates {
  L1,
  L2,
  L3,
  L4,
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
  void SetSuperState(SuperStates DesiredSuperState);

  // ALL WRONG - setpoint needs to be calculated
  units::turn_t L1Pos = units::turn_t(5);
  units::turn_t L2Pos = units::turn_t(10);
  units::turn_t L3Pos = units::turn_t(20);
  units::turn_t L4Pos = units::turn_t(30);

  units::turn_t setPoint;
  SuperStates currSuperState;

 private:
  ctre::phoenix6::hardware::TalonFX m_elevatorMotorLeft;
  ctre::phoenix6::hardware::TalonFX m_elevatorMotorRight;

  ctre::phoenix6::configs::TalonFXConfiguration m_elevatorConfig;
};