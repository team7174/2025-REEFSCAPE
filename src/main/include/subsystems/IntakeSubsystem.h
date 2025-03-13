#pragma once

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkFlex.h>

#include "Constants.h"
#include "subsystems/ElevatorSubsystem.h"

class IntakeSubsystem : public frc2::SubsystemBase
{
public:

  frc::XboxController *m_driveController;

  IntakeSubsystem(frc::XboxController *);
  ElevatorSubsystem *m_elevatorSubsystem;

  void Periodic() override;
  void Stop();
  // void rumbleController();

  bool FirstBeamBreakTriggered();
  bool SecondBeamBreakTriggered();
  bool CurrentSpiked();

  enum IntakeStates
  {
    coralIntake,
    algaeIntake,
    coralScore,
    algaeScore,
    hold
  };

  void SetIntakeState(IntakeStates desiredCoralState);

  units::second_t intakeTimeStamp;

  IntakeStates currentState = IntakeStates::coralIntake;

  void rumbleController();
  // private means only accessable in that respective file
private:
  rev::spark::SparkFlex m_coralIntakeMotor;
  rev::spark::SparkFlex m_algaeIntakeMotor;
  rev::spark::SparkFlex m_algaePivotMotor;
  rev::spark::SparkClosedLoopController m_algaePivotPID;
  rev::spark::SparkBaseConfig m_algaePivotConfig;

  frc::DigitalInput firstBeamBreak{IntakeConstants::firstIntakeBeamBreakID};
  frc::DigitalInput secondBeamBreak{IntakeConstants::secondIntakeBeamBreakID};

  double coralSpeed = 0.0;
  double algaeSpeed = 0.0;
};