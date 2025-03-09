// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc2/command/CommandScheduler.h>

frc::Transform3d frontRightTransform{
    frc::Translation3d(0.3175_m, -0.3048_m, 0.0_m), // X (forward), Y (right), Z (assumed 0)
    frc::Rotation3d(0_deg, 15_deg, 15_deg) // Start with no rotation
};

Robot::Robot()
    : kTagLayout(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)),
      visionSystem(kTagLayout, "FrontRight", frontRightTransform, "Camera2", frc::Transform3d{})
{
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();

  visionSystem.Update();
  auto robotPoses = visionSystem.GetRobotPoses();
  if (!robotPoses.empty())
  {
    // Define an example measurement standard deviation (tuned per robot)
    std::array<double, 3> visionStdDevs = {0.5, 0.5, 10.0}; // X, Y (meters), Theta (radians)

    m_container.drivetrain.SetVisionMeasurementStdDevs(visionStdDevs);
    m_container.drivetrain.AddVisionMeasurement(robotPoses[0].ToPose2d(), frc::Timer::GetFPGATimestamp());
  }

  /*
   * This example of adding Limelight is very simple and may not be sufficient for on-field use.
   * Users typically need to provide a standard deviation that scales with the distance to target
   * and changes with number of tags available.
   *
   * This example is sufficient to show that vision integration is possible, though exact implementation
   * of how to use vision should be tuned per-robot and to the team's specification.
   */
  if (kUseLimelight)
  {
    auto const driveState = m_container.drivetrain.GetState();
    auto const heading = driveState.Pose.Rotation().Degrees();
    auto const omega = driveState.Speeds.omega;

    LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
    auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (llMeasurement && llMeasurement->tagCount > 0 && units::math::abs(omega) < 2_tps)
    {
      m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
    }
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
  timer.Start();
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
