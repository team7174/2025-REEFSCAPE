// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <iostream>
#include "subsystems/CANdleSystem.h"
#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/FireAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/RgbFadeAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/TwinkleAnimation.h"
#include "ctre/phoenix/led/TwinkleOffAnimation.h"

using namespace ctre::phoenix::led;

CANdleSystem::CANdleSystem()
{
    ChangeAnimation(AnimationTypes::SetAll);
    CANdleConfiguration configAll{};
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType::GRB;
    configAll.brightnessScalar = 1.0;
    configAll.vBatOutputMode = VBatOutputMode::Modulated;
    m_candle.ConfigAllSettings(configAll, 100);
}

CANdleSystem::~CANdleSystem()
{
    if (m_toAnimate != NULL)
        delete m_toAnimate;
}

void CANdleSystem::SetColors()
{
    ChangeAnimation(AnimationTypes::SetAll);
}

void CANdleSystem::UpdateSetLed(std::function<double()> r, std::function<double()> g, std::function<double()> b, std::function<double()> modulatedVbat)
{
    m_r = g();
    m_g = r();
    m_b = b();
    m_modulatedOut = modulatedVbat();
    ChangeAnimation(AnimationTypes::SetAll);
}

/* Wrappers so we can access the CANdle from the subsystem */
double CANdleSystem::GetVbat() { return m_candle.GetBusVoltage(); }
double CANdleSystem::Get5V() { return m_candle.Get5VRailVoltage(); }
double CANdleSystem::GetCurrent() { return m_candle.GetCurrent(); }
double CANdleSystem::GetTemperature() { return m_candle.GetTemperature(); }
double CANdleSystem::GetModulatedOutput() { return m_candle.GetVBatModulation(); }
void CANdleSystem::ConfigBrightness(double percent) { m_candle.ConfigBrightnessScalar(percent, 0); }
void CANdleSystem::ConfigLos(bool disableWhenLos) { m_candle.ConfigLOSBehavior(disableWhenLos, 0); }
void CANdleSystem::ConfigLedType(LEDStripType type) { m_candle.ConfigLEDType(type, 0); }
void CANdleSystem::ConfigStatusLedBehavior(bool offWhenActive) { m_candle.ConfigStatusLedState(offWhenActive, 0); }

void CANdleSystem::ChangeAnimation(AnimationTypes toChange)
{
    m_currentAnimation = toChange;

    if (m_toAnimate != NULL)
        delete m_toAnimate;

    switch (toChange)
    {
    default:
    case SetAll:
        m_toAnimate = NULL;
        break;
    }
}

void CANdleSystem::Periodic()
{
    m_candle.SetLEDs(m_r, m_g, m_b);
    m_candle.ModulateVBatOutput(m_modulatedOut);
}

void CANdleSystem::SimulationPeriodic()
{
    // This method will be called once per scheduler run during simulation
}