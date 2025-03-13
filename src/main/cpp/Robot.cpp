// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandScheduler.h>

Robot::Robot()
{
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();

  auto visionEst = vision.GetEstimatedGlobalPose();
  if (visionEst.has_value()) {
    auto est = visionEst.value();
    auto estPose = est.estimatedPose.ToPose2d();
    auto estStdDevs = vision.GetEstimationStdDevs(estPose);
    std::array<double, 3> stdDevs = {estStdDevs(0), estStdDevs(1), estStdDevs(2)};
    frc::SmartDashboard::PutNumberArray("estDevs", stdDevs);
    frc::SmartDashboard::PutNumber("X", double(est.estimatedPose.ToPose2d().X()));
    frc::SmartDashboard::PutNumber("Y", double(est.estimatedPose.ToPose2d().Y()));
    //m_container.drivetrain.SetVisionMeasurementStdDevs(stdDevs);
    m_container.drivetrain.AddVisionMeasurement(est.estimatedPose.ToPose2d(), est.timestamp);
  }

  // auto backLeftVisionEst = vision.GetBackLeftEstimatedGlobalPose();
  // if (backLeftVisionEst.has_value())
  // {
  //   auto est = backLeftVisionEst.value();
  //   auto estPose = est.estimatedPose.ToPose2d();
  //   auto estStdDevs = vision.GetEstimationStdDevs(estPose, "BackLeft");

  //   std::array<double, 3> stdDevs = {estStdDevs(0), estStdDevs(1), estStdDevs(2)};
  //   m_container.drivetrain.SetVisionMeasurementStdDevs(stdDevs);

  //   m_container.drivetrain.AddVisionMeasurement(est.estimatedPose.ToPose2d(), est.timestamp);
  // }


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
