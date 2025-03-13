#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

using namespace subsystems;

void CommandSwerveDrivetrain::ConfigureAutoBuilder()
{
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        // Supplier of current robot pose
        [this] { return GetState().Pose; },
        // Consumer for seeding pose against auto
        [this](frc::Pose2d const &pose) { return ResetPose(pose); },
        // Supplier of current robot speeds
        [this] { return GetState().Speeds; },
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        [this](frc::ChassisSpeeds const &speeds, pathplanner::DriveFeedforwards const &feedforwards) {
            return SetControl(
                m_pathApplyRobotSpeeds.WithSpeeds(speeds)
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
            );
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // PID constants for translation
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            // PID constants for rotation
            pathplanner::PIDConstants{7.0, 0.0, 0.0}
        ),
        std::move(config),
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        [] {
            auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
            return alliance == frc::DriverStation::Alliance::kRed;
        },
        this // Subsystem for requirements
    );
}

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

frc::Pose2d CommandSwerveDrivetrain::ClosestAprilTag(frc::Pose2d robotPose)
{
    //Use the robot pose and return the closest AprilTag on a REEF
    std::vector<int> tagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};

    for (int tagID : tagIDs) {
        auto tagPose = VisionConstants::kTagLayout.GetTagPose(tagID);
        if (!tagPose) {
            continue;
        }
        frc::Pose2d tagPose2d(tagPose->X(), tagPose->Y(), frc::Rotation2d(tagPose->Rotation().Z()));
        double distance = robotPose.Translation().Distance(tagPose2d.Translation()).value();
        if (distance < minDistance) {
            minDistance = distance;
            closestTagID = tagID;
            closestTagPose = tagPose2d;
        }
    }
    return closestTagPose;
}

frc2::CommandPtr CommandSwerveDrivetrain::AutoAlign(ScoringOptions options)
{
    auto closestTagPose = ClosestAprilTag(TunerSwerveDrivetrain::GetState().Pose);

    double x1 = closestTagPose.X().value();
    double y1 = closestTagPose.Y().value();
    double z1 = closestTagPose.Rotation().Radians().value();

    double translatedX = x1 + (ScoringConstants::robotToReef * std::cos(z1));
    double translatedY = y1 + (ScoringConstants::robotToReef * std::sin(z1));

    switch (options) {
        case ScoringOptions::left:
        translatedX = translatedX + ((ScoringConstants::reefSpacing - ScoringConstants::scoringChuteOffset) * std::cos(z1 - M_PI_2));
        translatedY = translatedY + ((ScoringConstants::reefSpacing - ScoringConstants::scoringChuteOffset) * std::sin(z1 - M_PI_2));
        break;

        case ScoringOptions::right:
        translatedX = translatedX + ((ScoringConstants::reefSpacing + ScoringConstants::scoringChuteOffset) * std::cos(z1 + M_PI_2));
        translatedY = translatedY + ((ScoringConstants::reefSpacing + ScoringConstants::scoringChuteOffset) * std::sin(z1 + M_PI_2));
        break;

        case ScoringOptions::algae:
        translatedX = translatedX + ((ScoringConstants::scoringChuteOffset) * std::cos(z1 + M_PI_2));
        translatedY = translatedY + ((ScoringConstants::scoringChuteOffset) * std::sin(z1 + M_PI_2));
        break;
    }

    targetPose = frc::Pose2d{units::meter_t(translatedX), units::meter_t(translatedY), closestTagPose.Rotation()};

    pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
        3.0_mps, 4.0_mps_sq,
        540_deg_per_s, 720_deg_per_s_sq);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    frc2::CommandPtr pathfindingCommand = pathplanner::AutoBuilder::pathfindToPose(
        targetPose,
        constraints,
        0.0_mps // Goal end velocity in meters/sec
    );

    return pathfindingCommand;
}