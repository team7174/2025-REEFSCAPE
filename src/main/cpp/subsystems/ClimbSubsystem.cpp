
#include "subsystems/ClimbSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

ClimbSubsystem::ClimbSubsystem()
    : m_climbMotor(ClimbConstants::climbID) {
  auto &slot0Configs = m_climbConfig.Slot0;
  slot0Configs.kS = 0.05;  // Add 0.1 V output to overcome static friction
  slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  slot0Configs.kP = 0.025; // An error of 1 rps results in 0.11 V output
  slot0Configs.kI = 0;    // no output for integrated error
  slot0Configs.kD = 0;    // no output for error derivative

  auto &CurrLimit = m_climbConfig.CurrentLimits;
  CurrLimit.StatorCurrentLimit = 80_A;
  CurrLimit.StatorCurrentLimitEnable = true;

  auto &VoltLimit = m_climbConfig.Voltage;
  VoltLimit.PeakForwardVoltage = 12_V;
  VoltLimit.PeakReverseVoltage = -12_V;

  
  m_climbMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
  m_climbMotor.GetConfigurator().Apply(m_climbConfig);
  m_climbMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void ClimbSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Climb Position", (m_climbMotor.GetPosition().GetValueAsDouble()));
}

void ClimbSubsystem::SetClimbState(ClimbStates desiredState) {
  switch (desiredState) {
    case ClimbStates::in:
      setPoint = ClimbStates::in;
      break;
    case ClimbStates::out:
      setPoint = ClimbStates::out;
      break;
    default:
      setPoint = 0;
      break;
  }
  setPoint = std::clamp(setPoint, 0.0, 450.0);
  ctre::phoenix6::controls::PositionDutyCycle desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(setPoint)};
  desiredPos.WithEnableFOC(true);
  m_climbMotor.SetControl(desiredPos);
}

// void ClimbSubsystem::Stop() {
//   m_climbMotor.StopMotor();
// }

// bool ClimbSubsystem::IsAtSetpoint() {
//   return setPoint == m_climbMotor.GetPosition().GetValue();
// }