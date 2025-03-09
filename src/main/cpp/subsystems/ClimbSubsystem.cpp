
#include "subsystems/ClimbSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "ctre/phoenix6/controls/Follower.hpp"

ClimbSubsystem::ClimbSubsystem()
    : m_climbMotor(ClimbConstants::climbID) {
  auto &slot0Configs = m_climbConfig.Slot0;
  slot0Configs.kS = 0.6;   // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 11.0;  // A velocity target of 1 rps results in 0.12 V output
  // slot0Configs.kA = 0.025;                                                                                                                              // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = 0.0;  // An error of 1 rps results in 0.11 V output
  slot0Configs.kI = 0.0;  // no output for integrated error
  slot0Configs.kD = 0.0;  // no output for error derivative

  auto &CurrLimit = m_climbConfig.CurrentLimits;
  CurrLimit.StatorCurrentLimit = 30_A;
  CurrLimit.StatorCurrentLimitEnable = true;

  auto &VoltLimit = m_climbConfig.Voltage;
  VoltLimit.PeakForwardVoltage = 12_V;
  VoltLimit.PeakReverseVoltage = -12_V;

  m_climbMotor.GetConfigurator().Apply(m_climbConfig);
  m_climbMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void ClimbSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Climb Position", (m_climbMotor.GetPosition().GetValueAsDouble()));
}

void ClimbSubsystem::SetClimbState(ClimbState desiredState) {
  double position = 0.0;

  switch (desiredState) {
    case ClimbState::in:
      position = InPos;
      break;
    case ClimbState::out:
      position = OutPos;
      break;
    case ClimbState::hold:
      position = m_climbMotor.GetPosition().GetValueAsDouble();
      break;
    default:
      position = 0.0;
      break;
  }
  desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(position)};
  setPoint = units::turn_t(position);
}

void ClimbSubsystem::Stop() {
  m_climbMotor.StopMotor();
}

bool ClimbSubsystem::IsAtSetpoint() {
  return setPoint == m_climbMotor.GetPosition().GetValue();
}