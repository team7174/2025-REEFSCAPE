#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem(frc::XboxController *primaryController)
    : m_coralIntakeMotor(IntakeConstants::coralIntakeID, rev::spark::SparkFlex::MotorType::kBrushless), // Replace with your TalonFX device ID
                                                                                                        // m_algaeIntakeMotor(IntakeConstants::algaeIntakeID, rev::spark::SparkFlex::MotorType::kBrushless),
      m_algaePivotMotor(IntakeConstants::algaePivotID, rev::spark::SparkFlex::MotorType::kBrushless),
      m_algaePivotPID(m_algaePivotMotor.GetClosedLoopController()) // Replace with your TalonFX device ID
{
  m_driveController = primaryController;

    m_algaePivotConfig.closedLoop
      .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .P(0.2)
      .I(0)
      .D(0.1)
      .OutputRange(-1, 1)
      // Set PID values for velocity control in slot 1
      .P(0.0001, rev::spark::ClosedLoopSlot::kSlot1)
      .I(0, rev::spark::ClosedLoopSlot::kSlot1)
      .D(0, rev::spark::ClosedLoopSlot::kSlot1)
      .VelocityFF(1.0 / 5767, rev::spark::ClosedLoopSlot::kSlot1)
      .OutputRange(-1, 1, rev::spark::ClosedLoopSlot::kSlot1);

  m_algaePivotConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

  m_coralIntakeMotor.Configure(m_algaePivotConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                               rev::spark::SparkBase::PersistMode::kPersistParameters);

  m_algaePivotMotor.Configure(m_algaePivotConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kPersistParameters);

  m_algaePivotMotor.GetEncoder().SetPosition(0);

  // m_algaeIntakeMotor.Configure(m_algaePivotConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void IntakeSubsystem::Periodic()
{
  frc::SmartDashboard::PutBoolean("First Beam", FirstBeamBreakTriggered());
  frc::SmartDashboard::PutBoolean("Second Beam", SecondBeamBreakTriggered());
  frc::SmartDashboard::PutNumber("Pivot Rot", m_algaePivotMotor.GetEncoder().GetPosition());

  if (currentState == IntakeStates::coralIntake && FirstBeamBreakTriggered())
  {
    intakeTimeStamp = frc::Timer::GetFPGATimestamp();
    SetIntakeState(IntakeStates::hold);
  }

  if (currentState == IntakeStates::hold)
  {
    rumbleController();
    if (!FirstBeamBreakTriggered())
    {
      coralSpeed = -0.2;
    }
    else if (!SecondBeamBreakTriggered())
    {
      coralSpeed = 0.2;
    }
    else
    {
      coralSpeed = 0;
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
  // m_algaeIntakeMotor.Set(algaeSpeed);
}

bool IntakeSubsystem::FirstBeamBreakTriggered()
{
  return (!firstBeamBreak.Get());
}

bool IntakeSubsystem::SecondBeamBreakTriggered()
{
  return (!secondBeamBreak.Get());
}

bool IntakeSubsystem::CurrentSpiked()
{
  // WRONG CURRENT
  // return (m_algaeIntakeMotor.GetSupplyCurrent().GetValueAsDouble() > IntakeConstants::currentThreshold);
  return false;
}

void IntakeSubsystem::SetIntakeState(IntakeStates desiredIntakeState)
{
  currentState = desiredIntakeState;
  switch (desiredIntakeState)
  {
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
    coralSpeed = 0.75;
    break;
  case IntakeStates::coralScore:
    algaeSpeed = 0.0;
    coralSpeed = 0.4;
    break;
  default:
    algaeSpeed = 0.0;
    coralSpeed = 0.0;
    break;
  }
}

void IntakeSubsystem::rumbleController()
{
  if ((frc::Timer::GetFPGATimestamp() - intakeTimeStamp) < 1_s)
  {
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
  }
  else
  {
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  }
}

void IntakeSubsystem::setAlgaePivot(bool algae)
{
  if (algae)
  {
    m_algaePivotPID.SetReference(2,
                                        rev::spark::SparkBase::ControlType::kPosition,
                                        rev::spark::ClosedLoopSlot::kSlot0);  }
  else
  {
    m_algaePivotPID.SetReference(0,
                                        rev::spark::SparkBase::ControlType::kPosition,
                                        rev::spark::ClosedLoopSlot::kSlot0);  }
}