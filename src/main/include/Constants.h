#pragma once

#include <FieldConstants.h>

#include <numbers>

namespace ElevatorConstants {
constexpr int leftElevatorID = 60;
constexpr int rightElevatorID = 61;
// ALL WRONG NUMBERS
}  // namespace ElevatorConstants

namespace IntakeConstants {
// ALL WRONG NUMBERS
constexpr int coralIntakeID = 52;
constexpr int algaeIntakeID = 53;
constexpr int algaePivotID = 54;
constexpr int firstIntakeBeamBreakID = 0;
constexpr int secondIntakeBeamBreakID = 1;
constexpr int currentThreshold = 30;
constexpr units::second_t kEjectDuration = 4_s;
}  // namespace IntakeConstants

namespace ClimbConstants {
// ALL WRONG NUMBERS
constexpr int climbID = 66;
}  // namespace ClimbConstants

namespace ScoringConstants {
constexpr double reefSpacing = 0.1643126;
constexpr double scoringChuteOffset = 0.1778;
constexpr double robotToReef = 0.5;
}  // namespace ScoringConstants