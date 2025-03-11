#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "ctre/phoenix6/controls/Follower.hpp"
#include "RobotContainer.h" // Ensure this header file is included

ElevatorSubsystem::ElevatorSubsystem()
    : m_elevatorMotorLeft(ElevatorConstants::leftElevatorID), // Replace with your TalonFX device ID
      m_elevatorMotorRight(ElevatorConstants::rightElevatorID)
{
  auto &slot0Configs = m_elevatorConfig.Slot0;
  slot0Configs.kS = 0.6;  // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 11.0; // A velocity target of 1 rps results in 0.12 V output
  // slot0Configs.kA = 0.025;                                                                                                                              // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = 0.0; // An error of 1 rps results in 0.11 V output
  slot0Configs.kI = 0.0; // no output for integrated error
  slot0Configs.kD = 0.0; // no output for error derivative

  auto &CurrLimit = m_elevatorConfig.CurrentLimits;
  CurrLimit.StatorCurrentLimit = 30_A;
  CurrLimit.StatorCurrentLimitEnable = true;

  auto &VoltLimit = m_elevatorConfig.Voltage;
  VoltLimit.PeakForwardVoltage = 12_V;
  VoltLimit.PeakReverseVoltage = -12_V;

  auto &outputConfig = m_elevatorConfig.MotorOutput;

  outputConfig.Inverted = false;
  m_elevatorMotorLeft.GetConfigurator().Apply(m_elevatorConfig);

  outputConfig.Inverted = true;
  m_elevatorMotorRight.GetConfigurator().Apply(m_elevatorConfig);

  m_elevatorMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_elevatorMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void ElevatorSubsystem::Periodic()
{
  frc::SmartDashboard::PutNumber("Elevator Position", (m_elevatorMotorLeft.GetPosition().GetValueAsDouble()));
}

void ElevatorSubsystem::SetElevatorState(ElevatorStates desiredState)
{

  switch (desiredState)
  {
  case ElevatorStates::stow:
    setPoint = 0;
    break;
  case ElevatorStates::L1:
    setPoint = L1;
    break;
  case ElevatorStates::L2:
    setPoint = L2;

    if (RobotContainer::GetInstance().CoralMode)
    {
      setPoint += algaeOffset;
    }
    break;
  case ElevatorStates::L3:
    setPoint = L3;

    if (RobotContainer::GetInstance().CoralMode)
    {
      setPoint += algaeOffset;
    }
    break;
  case ElevatorStates::L4:
    setPoint = L4;
    break;
  case ElevatorStates::Barge:
    setPoint = Barge;
    break;
  case ElevatorStates::hold:
    setPoint = (m_elevatorMotorLeft.GetPosition().GetValueAsDouble() +
                m_elevatorMotorRight.GetPosition().GetValueAsDouble()) /
               2;
    break;
  default:
    setPoint = 0;
    break;
  }

  ctre::phoenix6::controls::PositionDutyCycle desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(setPoint)};
  m_elevatorMotorLeft.SetControl(desiredPos);
  m_elevatorMotorRight.SetControl(desiredPos);
}

void ElevatorSubsystem::Stop()
{
  m_elevatorMotorLeft.StopMotor();
  m_elevatorMotorRight.StopMotor();
}

bool ElevatorSubsystem::IsAtSetpoint()
{
  return setPoint == m_elevatorMotorLeft.GetPosition().GetValueAsDouble();
}