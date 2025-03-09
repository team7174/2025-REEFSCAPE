#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>

#include "subsystems/ElevatorSubsystem.h"
#include "Constants.h"


class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();
  ElevatorSubsystem *m_elevatorSubsystem;


  void Periodic() override;
  enum CoralStates {
    hold,
    intake,
    eject,
    slow
  };
  enum AlgaeStates {
    hold,
    intake,
    eject,
  };
  
  void SetCoralStates(CoralStates desiredCoralState);
  void SetAlgaeStates(AlgaeStates desiredAlgaeState);
  void Stop();
  void rumbleController();

  CoralStates currCoralState;
  AlgaeStates currAlgaeState;

  bool FirstBeamBreakTriggered();
  bool SecondBeamBreakTriggered();
  bool CurrentSpiked();

  units::second_t timeOriginal;

    //private means only accessable in that respective file 
 private:
  ctre::phoenix6::hardware::TalonFX m_coralIntakeMotor;
  ctre::phoenix6::hardware::TalonFX m_algaeIntakeMotor;
  

  ctre::phoenix6::configs::TalonFXConfiguration m_intakeConfig;

  double intakeSpeed;
  double algaeSpeed;

  frc::DigitalInput firstBeamBreak{IntakeConstants::firstIntakeBeamBreakID};
  frc::DigitalInput secondBeamBreak{IntakeConstants::secondIntakeBeamBreakID};   
  
};