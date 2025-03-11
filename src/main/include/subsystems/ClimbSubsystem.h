#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix6/TalonFX.hpp>

#include "Constants.h"

class ClimbSubsystem : public frc2::SubsystemBase {
 public:
  ClimbSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  enum ClimbStates {
    in,
    out,
    hold
  };

  void SetClimbState(ClimbStates DesiredClimbState);
  void Stop();
  bool IsAtSetpoint();

  //ALL WRONG - setpoint needs to be calculated
  units::angle::turn_t InPos = units::angle::turn_t(0);
  units::angle::turn_t OutPos = units::angle::turn_t(5);
  units::angle::turn_t setPoint = units::angle::turn_t(0);

  double holdVal = 0;

  ctre::phoenix6::controls::PositionDutyCycle desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(0)};


 private:
  ctre::phoenix6::hardware::TalonFX m_climbMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_climbConfig;
};