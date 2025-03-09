#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem()
    : m_coralIntakeMotor(IntakeConstants::coralIntakeID),  // Replace with your TalonFX device ID
      m_algaeIntakeMotor(IntakeConstants::algaeIntakeID)   // Replace with your TalonFX device ID
{
  auto &slot0Configs = m_intakeConfig.Slot0;
  slot0Configs.kS = 0.6;   // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 11.0;  // A velocity target of 1 rps results in 0.12 V output
  // slot0Configs.kA = 0.025;                                                                                                                              // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP = 0.0;  // An error of 1 rps results in 0.11 V output
  slot0Configs.kI = 0.0;  // no output for integrated error
  slot0Configs.kD = 0.0;  // no output for error derivative

  auto &CurrLimit = m_intakeConfig.CurrentLimits;
  CurrLimit.StatorCurrentLimit = 30_A;
  CurrLimit.StatorCurrentLimitEnable = true;

  auto &VoltLimit = m_intakeConfig.Voltage;
  VoltLimit.PeakForwardVoltage = 12_V;
  VoltLimit.PeakReverseVoltage = -12_V;

  m_coralIntakeMotor.GetConfigurator().Apply(m_intakeConfig);
  m_coralIntakeMotor.SetInverted(true);
  m_coralIntakeMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  m_algaeIntakeMotor.GetConfigurator().Apply(m_intakeConfig);
  m_algaeIntakeMotor.SetInverted(true);
  m_algaeIntakeMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void IntakeSubsystem::Periodic() {
  m_coralIntakeMotor.Set(intakeSpeed);
  m_algaeIntakeMotor.Set(algaeSpeed);

  if (currCoralState != CoralStates::hold && !SecondBeamBreakTriggered() && !FirstBeamBreakTriggered()) {
    SetCoralStates(CoralStates::intake);
  } else if (currCoralState == CoralStates::intake && FirstBeamBreakTriggered()) {
    SetCoralStates(CoralStates::slow);
  } else if ((currCoralState == CoralStates::intake || currCoralState == CoralStates::slow) && SecondBeamBreakTriggered()) {
    SetCoralStates(CoralStates::hold);
  } else if (currCoralState == CoralStates::hold && m_elevatorSubsystem->IsAtSetpoint()) {
    SetCoralStates(CoralStates::eject);
  }

  if (currAlgaeState == AlgaeStates::intake && CurrentSpiked()) {
    SetAlgaeStates(AlgaeStates::hold);
  } else if (currAlgaeState == AlgaeStates::hold && m_elevatorSubsystem->IsAtSetpoint()) {
    SetAlgaeStates(AlgaeStates::eject);
    timeOriginal = frc::Timer::GetFPGATimestamp();
  } else if (currAlgaeState == AlgaeStates::eject &&
             (frc::Timer::GetFPGATimestamp() - timeOriginal) > IntakeConstants::kEjectDuration) {
    SetAlgaeStates(AlgaeStates::intake);
  }
}

bool IntakeSubsystem::FirstBeamBreakTriggered() {
  return (!firstBeamBreak.Get());
}

bool IntakeSubsystem::SecondBeamBreakTriggered() {
  return (!secondBeamBreak.Get());
}

bool IntakeSubsystem::CurrentSpiked() {
  // WRONG CURRENT
  return (m_algaeIntakeMotor.GetSupplyCurrent().GetValueAsDouble() > IntakeConstants::currentThreshold);
}

void IntakeSubsystem::SetCoralStates(CoralStates desiredCoralState) {
  currCoralState = desiredCoralState;
  switch (desiredCoralState) {
    case CoralStates::hold:
      intakeSpeed = 0.0;
      break;
    case CoralStates::intake:
      intakeSpeed = 1.0;
      break;
    case CoralStates::eject:
      intakeSpeed = -1.0;
      break;
    case CoralStates::slow:
      intakeSpeed = 0.5;
      break;
    default:
      intakeSpeed = 0.0;
      break;
  }
}

void IntakeSubsystem::SetAlgaeStates(AlgaeStates desiredAlgaeState) {
  currAlgaeState = desiredAlgaeState;
  switch (desiredAlgaeState) {
    case AlgaeStates::hold:
      algaeSpeed = 0.0;
      break;
    case AlgaeStates::intake:
      algaeSpeed = 1.0;
      break;
    case AlgaeStates::eject:
      algaeSpeed = -1.0;
      break;
    default:
      algaeSpeed = 0.0;
      break;
  }
}
