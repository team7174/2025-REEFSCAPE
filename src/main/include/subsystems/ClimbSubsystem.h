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
    in = 0,
    out = 450,
    hold = 0
  };

  void SetClimbState(ClimbStates DesiredClimbState);
  void Stop();
  bool IsAtSetpoint();

  double holdVal = 0;
  
  double setPoint;

 private:
  ctre::phoenix6::hardware::TalonFX m_climbMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_climbConfig;
};