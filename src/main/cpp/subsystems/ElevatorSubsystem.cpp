#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "ctre/phoenix6/controls/Follower.hpp"
#include "RobotContainer.h" // Ensure this header file is included

ElevatorSubsystem::ElevatorSubsystem()
    : m_elevatorMotorLeft(ElevatorConstants::leftElevatorID), // Replace with your TalonFX device ID
      m_elevatorMotorRight(ElevatorConstants::rightElevatorID),
      profiledController(
          0.5,  // Placeholder for proportional gain
          0.0,  // Placeholder for integral gain
          0.0,  // Placeholder for derivative gain
          frc::TrapezoidProfile<units::turns>::Constraints(500_tps, 100_tr_per_s_sq))
{
  auto &slot0Configs = m_elevatorConfig.Slot0;
  slot0Configs.kS = 0.5;  // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  // slot0Configs.kA = 0.025;                                                                                                                              // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = 0.5; // An error of 1 rps results in 0.11 V output
  slot0Configs.kI = 0.0; // no output for integrated error
  slot0Configs.kD = 0.0; // no output for error derivative

  auto &motionMagicConfigs = m_elevatorConfig.MotionMagic;
  motionMagicConfigs.MotionMagicCruiseVelocity = 500_tps; // Target cruise velocity of 80 rps
  motionMagicConfigs.MotionMagicAcceleration = 1_tr_per_s_sq;  // Target acceleration of 160 rps/s (0.5 seconds)
  motionMagicConfigs.MotionMagicJerk = 500_tr_per_s_cu;         // Target jerk of 1600 rps/s/s (0.1 seconds)

  auto &CurrLimit = m_elevatorConfig.CurrentLimits;
  CurrLimit.StatorCurrentLimit = 80_A;
  CurrLimit.StatorCurrentLimitEnable = true;

  auto &VoltLimit = m_elevatorConfig.Voltage;
  VoltLimit.PeakForwardVoltage = 12_V;
  VoltLimit.PeakReverseVoltage = -12_V;

  auto &outputConfig = m_elevatorConfig.MotorOutput;

  outputConfig.Inverted = false;
  m_elevatorMotorLeft.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
  m_elevatorMotorLeft.GetConfigurator().Apply(m_elevatorConfig);

  outputConfig.Inverted = true;
  m_elevatorMotorRight.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
  m_elevatorMotorRight.GetConfigurator().Apply(m_elevatorConfig);

  m_elevatorMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  m_elevatorMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void ElevatorSubsystem::Periodic()
{
  frc::SmartDashboard::PutNumber("Elevator Position", (m_elevatorMotorLeft.GetPosition().GetValueAsDouble()));

  // m_elevatorMotorLeft.Set(profiledController.Calculate(units::angle::turn_t(m_elevatorMotorLeft.GetPosition().GetValueAsDouble())));
  // m_elevatorMotorRight.Set(profiledController.Calculate(units::angle::turn_t(m_elevatorMotorRight.GetPosition().GetValueAsDouble())));
}

void ElevatorSubsystem::SetElevatorState(ElevatorStates desiredState, bool algae)
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

    if (algae)
    {
      setPoint += algaeOffset;
    }
    break;
  case ElevatorStates::L3:
    setPoint = L3;

    if (algae)
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

  setPoint = std::clamp(setPoint, 0.0, 140.0);
  ctre::phoenix6::controls::PositionDutyCycle desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(setPoint)};
  m_elevatorMotorLeft.SetControl(desiredPos);
  m_elevatorMotorRight.SetControl(desiredPos);
  //profiledController.SetGoal(units::turn_t(setPoint));
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