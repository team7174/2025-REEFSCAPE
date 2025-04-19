#pragma once

#include <FieldConstants.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <numbers>

namespace ElevatorConstants
{
    constexpr int leftElevatorID = 60;
    constexpr int rightElevatorID = 61;
    // ALL WRONG NUMBERS
} // namespace ElevatorConstants

namespace IntakeConstants
{
    // ALL WRONG NUMBERS
    constexpr int coralIntakeID = 52;
    constexpr int algaePivotID = 54;
    constexpr int firstIntakeBeamBreakID = 0;
    constexpr int secondIntakeBeamBreakID = 1;
    constexpr int currentThreshold = 30;
    constexpr units::second_t kEjectDuration = 4_s;
} // namespace IntakeConstants

namespace ClimbConstants
{
    constexpr int climbID = 62;
    constexpr int algaeIntakeID = 59;
} // namespace ClimbConstants

namespace ScoringConstants
{
    constexpr double reefSpacing = 0.1643126;
    constexpr double scoringChuteOffset = 0.1643126;
    constexpr double robotToReef = 0.4572;
} // namespace ScoringConstants

namespace VisionConstants
{
    inline const frc::AprilTagFieldLayout kTagLayout{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded)};

    inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
    inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};

    inline constexpr std::string_view kCameraName{"FrontRight"};

    inline const frc::Transform3d kRobotToBackLeftCam(
        frc::Translation3d(-0.3048_m, -0.3048_m, 0.2076_m), // X: back, Y: left, Z: height
        frc::Rotation3d(
            units::degree_t(0),  // Roll (no sideways tilt)
            units::degree_t(15), // Pitch (tilt up)
            units::degree_t(165) // Yaw (facing backward, 180° - 15° left)
            ));

    inline const frc::Transform3d kRobotToFrontRightCam(
        frc::Translation3d(0.3048_m, -0.3048_m, 0.2076_m), // X: forward, Y: right, Z: height
        frc::Rotation3d(
            units::degree_t(0),  // Roll (no sideways tilt)
            units::degree_t(15), // Pitch (tilt up)
            units::degree_t(15) // Yaw (turn left)
            ));
}