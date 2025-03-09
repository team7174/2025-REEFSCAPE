#include <subsystems/IntakeSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem()
    : m_coralIntakeMotor(IntakeConstants::coralIntakeID), // Replace with your TalonFX device ID
      m_algaeIntakeMotor(IntakeConstants::algaeIntakeID) // Replace with your TalonFX device ID
{
    auto &slot0Configs = m_intakeConfig.Slot0;
    slot0Configs.kS = 0.6;  // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 11.0; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.025;                                                                                                                              // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0; // no output for error derivative

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



void IntakeSubsystem::Periodic()
{
    m_coralIntakeMotor.Set(intakeSpeed);
    m_algaeIntakeMotor.Set(algaeSpeed);

    //always in intake state if nothing is inside
    if(!SecondBeamBreakTriggered() && !FirstBeamBreakTriggered() && currCoralState != CoralStates::hold){
        SetCoralStates(CoralStates::intake);
    }

    //if the coral is in the intake state and the beam break is triggered, slow down the intake
    if (currCoralState == CoralStates::intake && (FirstBeamBreakTriggered() || SecondBeamBreakTriggered())){
        SetCoralStates(CoralStates::slow);
    }

    //get the coral to be perfectly in the middle of first + second beam break
    if (currCoralState == CoralStates::slow && SecondBeamBreakTriggered()){
        SetCoralStates(CoralStates::hold);
    }

    //if there is a coral there and the elevator is at the setpoint, eject the coral
    if (currCoralState == CoralStates::hold && m_elevatorSubsystem->IsAtSetpoint()){
        SetCoralStates(CoralStates::eject);
    }

    //if the algae is in the intake state and the current spiked, stop the intake
    if (currAlgaeState == AlgaeStates::intake && CurrentSpiked()){
        SetAlgaeStates(AlgaeStates::hold);        
    }

    //algae is being held, elevator gets to setpoint, eject the algae
    if (currAlgaeState == AlgaeStates::hold && m_elevatorSubsystem->IsAtSetpoint()){
        SetAlgaeStates(AlgaeStates::eject);
        timeOriginal = frc::Timer::GetFPGATimestamp();
    }

    //algae is being ejected, wait 4 seconds, then turn back to intake mode
    //SHOULD PROBABLY BE LESS THAN 4 SECONDS
    if (currAlgaeState == eject && (frc::Timer::GetFPGATimestamp() - timeOriginal) > 4.0_s){
        SetAlgaeStates(AlgaeStates::intake);
    }
    
}

bool IntakeSubsystem::FirstBeamBreakTriggered() {
    return(!firstBeamBreak.Get());
}

bool IntakeSubsystem::SecondBeamBreakTriggered() {
    return(!secondBeamBreak.Get());
}

bool IntakeSubsystem::CurrentSpiked() {
    //WRONG CURRENT
    return(m_algaeIntakeMotor.GetSupplyCurrent().GetValueAsDouble() > 30.0);
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
 