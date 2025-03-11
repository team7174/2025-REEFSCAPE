#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem(frc::XboxController *primaryController)
    : m_coralIntakeMotor(IntakeConstants::coralIntakeID),  // Replace with your TalonFX device ID
      m_algaeIntakeMotor(IntakeConstants::algaeIntakeID),
      m_algaePivotMotor(IntakeConstants::algaePivotID)   // Replace with your TalonFX device ID
{
  m_driveController = primaryController;
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
  m_coralIntakeMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  m_algaeIntakeMotor.GetConfigurator().Apply(m_intakeConfig);
  m_algaeIntakeMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void IntakeSubsystem::Periodic() {
  if (currentState == IntakeStates::coralIntake && (FirstBeamBreakTriggered() || SecondBeamBreakTriggered())) {
    intakeTimeStamp = frc::Timer::GetFPGATimestamp();
    coralSpeed = 0.5;
  }

  if (coralSpeed == 0.5) {
    rumbleController();
    if (!FirstBeamBreakTriggered()) {
      coralSpeed = -0.2;
    } else {
      coralSpeed = 0.0;
    }
  }

  if (currentState == IntakeStates::coralScore)
  {
    if (!FirstBeamBreakTriggered() && !SecondBeamBreakTriggered())
    {
      SetIntakeState(IntakeStates::coralIntake);
    }
  }
  
  m_coralIntakeMotor.Set(coralSpeed);
  m_algaeIntakeMotor.Set(algaeSpeed);
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

void IntakeSubsystem::SetIntakeState(IntakeStates desiredIntakeState) {
  currentState = desiredIntakeState;
  switch (desiredIntakeState) {
    case IntakeStates::algaeIntake:
      algaeSpeed = 0.0;
      coralSpeed = 0.0;
      break;
    case IntakeStates::algaeScore:
      algaeSpeed = 0.0;
      coralSpeed = 0.0;
      break;
    case IntakeStates::coralIntake:
      algaeSpeed = 0.0;
      coralSpeed = 0.0;
      break;
    case IntakeStates::coralScore:
      algaeSpeed = 0.0;
      coralSpeed = 0.0;
      break;
    default:
      algaeSpeed = 0.0;
      coralSpeed = 0.0;
      break;
  }
}

void IntakeSubsystem::rumbleController() {
  if ((frc::Timer::GetFPGATimestamp() - intakeTimeStamp) < 1_s) {
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
  } else {
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  }
}