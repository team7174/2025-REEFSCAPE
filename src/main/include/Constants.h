#pragma once

#include <numbers>
#include <FieldConstants.h>

namespace ElevatorConstants {
    constexpr int leftElevatorID = 60;
    constexpr int rightElevatorID = 61;
    //ALL WRONG NUMBERS
    constexpr units::turn_t algaeOffset = units::turn_t(1);
}

namespace IntakeConstants {
    // ALL WRONG NUMBERS
    constexpr int coralIntakeID = 62;
    constexpr int algaeIntakeID = 63;
    constexpr int firstIntakeBeamBreakID = 64;
    constexpr int secondIntakeBeamBreakID = 65;
}

namespace ClimbConstants {
    // ALL WRONG NUMBERS
    constexpr int climbID = 66;
}

namespace ScoringConstants {
    constexpr double reefSpacing = 0.1643126;
    constexpr double scoringChuteOffset = 0.1778;
    constexpr double robotToReef = 0.5;
}