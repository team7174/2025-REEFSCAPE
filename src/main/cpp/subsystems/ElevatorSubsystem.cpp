#include "subsystems/ElevatorSubsystem.h"
#include "ctre/phoenix6/controls/Follower.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

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

    m_elevatorMotorLeft.GetConfigurator().Apply(m_elevatorConfig);
    m_elevatorMotorLeft.SetInverted(true);
    m_elevatorMotorRight.GetConfigurator().Apply(m_elevatorConfig);

    m_elevatorMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_elevatorMotorRight.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}       

void ElevatorSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Elevator Position", (m_elevatorMotorLeft.GetPosition().GetValueAsDouble()));
}

void ElevatorSubsystem::SetSuperState(SuperStates DesiredSuperState)
{
    currSuperState = DesiredSuperState;
    switch (DesiredSuperState)
    {
    case SuperStates::coral:
        // ALL WRONG - setpoint needs to be calculated
        L1Pos = L1Pos;
        L2Pos = L2Pos;
        L3Pos = L3Pos;
        L4Pos = L4Pos;
        break;

    case SuperStates::algae:
        // ALL WRONG - setpoint needs to be calculated
        L1Pos = L1Pos - ElevatorConstants::algaeOffset;
        L2Pos = L2Pos - ElevatorConstants::algaeOffset;
        L3Pos = L3Pos - ElevatorConstants::algaeOffset;
        L4Pos = L4Pos - ElevatorConstants::algaeOffset;
        break;
    }
};

void ElevatorSubsystem::SetElevatorState(ElevatorStates DesiredElevatorState)
{
    switch (DesiredElevatorState)
    {
    case ElevatorStates::L1:
        // ALL WRONG - setpoint needs to be calculated
        desiredPos = ctre::phoenix6::controls::PositionDutyCycle{L1Pos};
        setPoint = L1Pos;
        break;

    case ElevatorStates::L2:
        // ALL WRONG - setpoint needs to be calculated
        desiredPos = ctre::phoenix6::controls::PositionDutyCycle{L2Pos};
        setPoint = L2Pos;
        break;

    case ElevatorStates::L3:
        // ALL WRONG - setpoint needs to be calculated
        desiredPos = ctre::phoenix6::controls::PositionDutyCycle{L3Pos};
        setPoint = L3Pos;
        break;

    case ElevatorStates::L4:
        // ALL WRONG - setpoint needs to be calculated
        desiredPos = ctre::phoenix6::controls::PositionDutyCycle{L4Pos};
        setPoint = L4Pos;
        break;

    case ElevatorStates::hold:
        holdVal = (m_elevatorMotorLeft.GetPosition().GetValueAsDouble() + m_elevatorMotorRight.GetPosition().GetValueAsDouble()) / 2;
        desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(holdVal)};
        setPoint = units::turn_t(holdVal);
        break;

    default:
        desiredPos = ctre::phoenix6::controls::PositionDutyCycle{units::turn_t(0)};
        setPoint = units::turn_t(units::turn_t(0));
        break;
    }

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
    return setPoint == m_elevatorMotorLeft.GetPosition().GetValue();
}