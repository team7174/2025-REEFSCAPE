#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem(frc::XboxController *primaryController)
    : m_coralIntakeMotor(IntakeConstants::coralIntakeID, rev::spark::SparkFlex::MotorType::kBrushless),  // Replace with your TalonFX device ID
      m_algaeIntakeMotor(IntakeConstants::algaeIntakeID, rev::spark::SparkFlex::MotorType::kBrushless)
      //m_algaePivotMotor(IntakeConstants::algaePivotID)   // Replace with your TalonFX device ID
{
  m_driveController = primaryController;
}

void IntakeSubsystem::Periodic() {
  frc::SmartDashboard::PutBoolean("First Beam", FirstBeamBreakTriggered());
  frc::SmartDashboard::PutBoolean("Second Beam", SecondBeamBreakTriggered());

  if (currentState == IntakeStates::coralIntake && FirstBeamBreakTriggered()) {
    intakeTimeStamp = frc::Timer::GetFPGATimestamp();
    SetIntakeState(IntakeStates::hold);
  }

  if (currentState == IntakeStates::hold) {
    rumbleController();
    if (!FirstBeamBreakTriggered()) {
      coralSpeed = -0.2;
    } else if (!SecondBeamBreakTriggered()){
      coralSpeed = 0.2;
    }
    else {
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
  
  m_coralIntakeMotor.Set(-coralSpeed);
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
  //return (m_algaeIntakeMotor.GetSupplyCurrent().GetValueAsDouble() > IntakeConstants::currentThreshold);
  return false;
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
      coralSpeed = 1.0;
      break;
    case IntakeStates::coralScore:
      algaeSpeed = 0.0;
      coralSpeed = 1.0;
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