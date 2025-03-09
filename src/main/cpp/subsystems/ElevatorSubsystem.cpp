#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "ctre/phoenix6/controls/Follower.hpp"

ElevatorSubsystem::ElevatorSubsystem()
    : m_elevatorMotorLeft(ElevatorConstants::leftElevatorID),  // Replace with your TalonFX device ID
      m_elevatorMotorRight(ElevatorConstants::rightElevatorID) {
  auto &slot0Configs = m_elevatorConfig.Slot0;
  slot0Configs.kS = 0.6;   // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 11.0;  // A velocity target of 1 rps results in 0.12 V output
  // slot0Configs.kA = 0.025;                                                                                                                              // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = 0.0;  // An error of 1 rps results in 0.11 V output
  slot0Configs.kI = 0.0;  // no output for integrated error
  slot0Configs.kD = 0.0;  // no output for error derivative

  auto &CurrLimit = m_elevatorConfig.CurrentLimits;
  CurrLimit.StatorCurrentLimit = 30_A;
  CurrLimit.StatorCurrentLimitEnable = true;

  auto &VoltLimit = m_elevatorConfig.Voltage;
  VoltLimit.PeakForwardVoltage = 12_V;
  VoltLimit.PeakReverseVoltage = -12_V;

  m_elevatorMotorRight.GetConfigurator().Apply(m_elevatorConfig);

  auto &outputConfig = m_elevatorConfig.MotorOutput;
  outputConfig.Inverted = true;
  m_elevatorMotorLeft.GetConfigurator().Apply(m_elevatorConfig);

  m_elevatorMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_elevatorMotorLeft.SetInverted(false);
  m_elevatorMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_elevatorMotorRight.SetInverted(true);
}

void ElevatorSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Elevator Position", (m_elevatorMotorLeft.GetPosition().GetValueAsDouble()));
}

void ElevatorSubsystem::SetSuperState(SuperStates DesiredSuperState) {
  currSuperState = DesiredSuperState;
};

void ElevatorSubsystem::SetElevatorState(ElevatorState desiredState) {
  double offset = (currSuperState == ElevatorState::algae) ? ElevatorConstants::algaeOffset : 0.0;
  double position = 0.0;

  switch (desiredState) {
    case ElevatorState::L1:
      position = L1Pos - offset;
      break;
    case ElevatorState::L2:
      position = L2Pos - offset;
      break;
    case ElevatorState::L3:
      position = L3Pos - offset;
      break;
    case ElevatorState::L4:
      position = L4Pos - offset;
      break;
    case ElevatorState::hold:
      position = (m_elevatorMotorLeft.GetPosition().GetValueAsDouble() +
                  m_elevatorMotorRight.GetPosition().GetValueAsDouble()) /
                 2;
      break;
    default:
      position = 0.0;
      break;
  }

  ctre::phoenix6::controls::PositionDutyCycle desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(position)};
  setPoint = units::turn_t(position);

  m_elevatorMotorLeft.SetControl(desiredPos);
  m_elevatorMotorRight.SetControl(desiredPos);
}

void ElevatorSubsystem::Stop() {
  m_elevatorMotorLeft.StopMotor();
  m_elevatorMotorRight.StopMotor();
}

bool ElevatorSubsystem::IsAtSetpoint() {
  return setPoint == m_elevatorMotorLeft.GetPosition().GetValue();
}